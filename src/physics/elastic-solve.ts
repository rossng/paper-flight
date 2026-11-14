import type { Face } from '../geometry/dart';
import type { HingeBatch } from './constraints';
import type { MembraneBatch } from './membrane';

/** Direct solve of a substep's elastic response.
 *
 * Gauss–Seidel sweeps over stiff membrane and hinge constraints settle on a softer sheet
 * than the model describes, and more sweeps do not fix it (docs/reference-solver.md). This
 * takes Newton steps on the same implicit-Euler problem the sweeps approximate,
 *   minimise  |x − x̃|²_M / 2dt²  +  E_membrane(x)  +  E_hinge(x),
 * with a sparse LDLᵀ factorisation whose ordering and fill are fixed by the mesh. It then
 * hands the sweeps positions and multipliers that already satisfy every elastic constraint,
 * so they only have to reconcile contact. */
export class ElasticSolver {
  private n:number; private perm:Int32Array;
  private blocks:Float64Array; private source:Int32Array;
  private ap:Int32Array; private ai:Int32Array; private ax:Float64Array;
  private lp:Int32Array; private parent:Int32Array; private lnz:Int32Array; private li:Int32Array; private lx:Float64Array;
  private d:Float64Array; private y:Float64Array; private pattern:Int32Array; private flag:Int32Array;
  private diagonal:Int32Array; private faceSlots:Int32Array; private hingeSlots:Int32Array;
  private gradient:Float64Array; private work:Float64Array; private step:Float64Array; private trial:Float64Array; private carried:Float64Array; private carriedStep=0;
  constructor(count:number,faces:Face[],hinges:{ids:number[]}[]){
    this.n=count;
    const pairs:[number,number][]=[];
    for(const f of faces)for(let a=0;a<3;a++)for(let b=a+1;b<3;b++)pairs.push([f[a],f[b]]);
    for(const h of hinges)for(let a=0;a<4;a++)for(let b=a+1;b<4;b++)pairs.push([h.ids[a],h.ids[b]]);
    const perm=this.perm=minimumDegree(count,pairs);
    const columns:number[][]=Array.from({length:count},(_,j)=>[j]);
    for(const [i,j] of pairs){const a=Math.min(perm[i],perm[j]),b=Math.max(perm[i],perm[j]);if(a!==b)columns[b].push(a);}
    const slots=new Map<number,number>();let blockCount=0;
    columns.forEach((column,j)=>{
      const rows=[...new Set(column)].sort((a,b)=>a-b);columns[j]=rows;
      for(const i of rows)slots.set(i*count+j,blockCount++);
    });
    // Upper triangle of the permuted matrix, by scalar column.
    const size=3*count,ap=this.ap=new Int32Array(size+1),ai:number[]=[],source:number[]=[];
    for(let j=0;j<count;j++)for(let b=0;b<3;b++){
      for(const i of columns[j]){
        const block=9*slots.get(i*count+j)!;
        if(i<j)for(let a=0;a<3;a++){ai.push(3*i+a);source.push(block+3*a+b);}
        else for(let a=0;a<=b;a++){ai.push(3*j+a);source.push(block+3*a+b);}
      }
      ap[3*j+b+1]=ai.length;
    }
    this.ai=Int32Array.from(ai);this.source=Int32Array.from(source);this.ax=new Float64Array(ai.length);this.blocks=new Float64Array(9*blockCount);
    // Elimination tree and column counts.
    const parent=this.parent=new Int32Array(size).fill(-1),lnz=this.lnz=new Int32Array(size),flag=this.flag=new Int32Array(size);
    for(let k=0;k<size;k++){
      flag[k]=k;
      for(let p=ap[k];p<ap[k+1];p++){
        let i=this.ai[p];
        if(i<k)for(;flag[i]!==k;i=parent[i]){if(parent[i]===-1)parent[i]=k;lnz[i]++;flag[i]=k;}
      }
    }
    const lp=this.lp=new Int32Array(size+1);for(let k=0;k<size;k++)lp[k+1]=lp[k]+lnz[k];
    this.li=new Int32Array(lp[size]);this.lx=new Float64Array(lp[size]);
    this.d=new Float64Array(size);this.y=new Float64Array(size);this.pattern=new Int32Array(size);
    this.gradient=new Float64Array(size);this.work=new Float64Array(size);this.step=new Float64Array(size);this.trial=new Float64Array(size);this.carried=new Float64Array(size);
    // Where each element's vertex pairs land: block offset, negative (as ~offset) when the
    // pair is stored transposed.
    const slot=(i:number,j:number)=>{const a=perm[i],b=perm[j];return a<=b?9*slots.get(a*count+b)!:~(9*slots.get(b*count+a)!);};
    this.diagonal=Int32Array.from({length:count},(_,i)=>slot(i,i));
    const collect=(ids:number[])=>{const out:number[]=[];for(let a=0;a<ids.length;a++)for(let b=a;b<ids.length;b++)out.push(slot(ids[a],ids[b]));return out;};
    this.faceSlots=Int32Array.from(faces.flatMap(collect));this.hingeSlots=Int32Array.from(hinges.flatMap(h=>collect(h.ids)));
  }
  /** Replace `x` (the unconstrained prediction) with the elastic implicit-Euler solution
   * and set the sweeps' multipliers to match. `alpha` values are compliance/dt². Newton
   * steps with a backtracking line search until the largest update is below `tolerance`
   * metres. Returns false, leaving the prediction untouched, if `iterations` steps do not
   * get there. */
  solve(x:Float64Array,masses:ArrayLike<number>,dt:number,membranes:MembraneBatch,hinges:HingeBatch,iterations:number,tolerance=1e-6){
    const size=3*this.n,g=this.gradient,predicted=this.work,step=this.step,trial=this.trial;
    predicted.set(x.subarray(0,size));
    // Start from the previous substep's elastic correction: loads change slowly, so the
    // sheet's response to them does too. It scales with dt².
    const carried=this.carried,ratio=this.carriedStep>0?(dt*dt)/(this.carriedStep*this.carriedStep):0;
    if(ratio>0){
      // Keep the carried start only if it is better than the plain prediction; after an
      // impact it need not be.
      for(let k=0;k<size;k++)x[k]+=ratio*carried[k];
      if(!(this.assemble(x,masses,dt,membranes,hinges,false)<=this.assemble(predicted,masses,dt,membranes,hinges,false)))x.set(predicted);
    }
    let converged=false;
    for(let iteration=0;iteration<iterations;iteration++){
      // One factorisation serves a solve's first Newton steps (the shape barely changes
      // within a substep) and is renewed if they have not converged. It is not kept between
      // substeps: the membrane's stiff directions turn with the sheet.
      const renew=iteration%RENEW===0;
      const energy=this.assemble(x,masses,dt,membranes,hinges,true,renew);
      if(renew&&!this.factor())break;
      this.backSolve(g,step);
      let slope=0,largest=0;
      for(let k=0;k<size;k++){slope-=g[k]*step[k];const move=Math.abs(step[k]);if(move>largest)largest=move;}
      // Backtrack until the incremental potential decreases enough; a step that cannot
      // decrease it (or a non-finite one) is not taken.
      let length=1,accepted=largest<tolerance;
      if(accepted){converged=true;break;}
      for(;!accepted&&length>=1e-3;length/=2){
        for(let k=0;k<size;k++)trial[k]=x[k]-length*step[k];
        if(this.assemble(trial,masses,dt,membranes,hinges,false)<=energy+1e-4*length*slope){accepted=true;break;}
      }
      if(!accepted||!(largest<1e3))break;
      x.set(trial);
      if(largest*length<tolerance){converged=true;break;}
    }
    if(!converged){
      // A partly converged shape carries unbalanced stiff forces. Hand the sweeps the plain
      // prediction instead, as if this solve had not run.
      x.set(predicted);membranes.lambda.fill(0);hinges.lambda.fill(0);this.carriedStep=0;
      return false;
    }
    for(let k=0;k<size;k++)carried[k]=x[k]-predicted[k];
    this.carriedStep=dt;
    this.multipliers(x,membranes,hinges);
    return true;
  }
  /** Forget the carried correction (after a reset or any discontinuity in the state). */
  reset(){this.carriedStep=0;}
  /** Incremental potential at `x`; with `derivatives`, also its gradient and (Gauss–Newton
   * for hinges, positive part for membranes) Hessian blocks. */
  private assemble(x:Float64Array,masses:ArrayLike<number>,dt:number,membranes:MembraneBatch,hinges:HingeBatch,derivatives:boolean,hessian=derivatives){
    const n=this.n,blocks=this.blocks,g=this.gradient,predicted=this.work,inverse=1/(dt*dt);
    const faces=membranes.faces,md=membranes.md,cd=membranes.cd,faceSlots=this.faceSlots,hingeSlots=this.hingeSlots;
    const ids=hinges.ids,hingeCount=hinges.lambda.length,faceCount=faces.length/3;
    const gradients=hingeGradients,hingeIds=hingeIdScratch;
    let energy=0;
    if(hessian)blocks.fill(0);
    for(let i=0;i<n;i++){
      const m=masses[i]*inverse,k=3*i,dx=x[k]-predicted[k],dy=x[k+1]-predicted[k+1],dz=x[k+2]-predicted[k+2];
      energy+=m*(dx*dx+dy*dy+dz*dz)/2;
      if(derivatives){g[k]=m*dx;g[k+1]=m*dy;g[k+2]=m*dz;if(hessian){const o=this.diagonal[i];blocks[o]+=m;blocks[o+4]+=m;blocks[o+8]+=m;}}
    }
    for(let f=0;f<faceCount;f++){
      const t=3*f,a=3*faces[t],b=3*faces[t+1],c=3*faces[t+2];
      const m0=md[t],m1=md[t+1],m2=md[t+2],c0=cd[t],c1=cd[t+1],c2=cd[t+2];
      const ux=m0*x[a]+m1*x[b]+m2*x[c],uy=m0*x[a+1]+m1*x[b+1]+m2*x[c+1],uz=m0*x[a+2]+m1*x[b+2]+m2*x[c+2];
      const vx=c0*x[a]+c1*x[b]+c2*x[c],vy=c0*x[a+1]+c1*x[b+1]+c2*x[c+1],vz=c0*x[a+2]+c1*x[b+2]+c2*x[c+2];
      const s0=(ux*ux+uy*uy+uz*uz-1)/2,s1=(vx*vx+vy*vy+vz*vz-1)/2,s2=ux*vx+uy*vy+uz*vz;
      // Stiffness is 1/(alpha dt²).
      const k0=inverse/membranes.alpha[t],k1=inverse/membranes.alpha[t+1],k2=inverse/membranes.alpha[t+2];
      energy+=(k0*s0*s0+k1*s1*s1+k2*s2*s2)/2;
      if(!derivatives)continue;
      const f0=k0*s0,f1=k1*s1,f2=k2*s2,tensionU=f0>0?f0:0,tensionV=f1>0?f1:0;
      memM[0]=m0;memM[1]=m1;memM[2]=m2;memC[0]=c0;memC[1]=c1;memC[2]=c2;memV[0]=a;memV[1]=b;memV[2]=c;
      let pair=6*f;
      for(let i=0;i<3;i++){
        const mi=memM[i],ci=memC[i],o=memV[i];
        const six=mi*vx+ci*ux,siy=mi*vy+ci*uy,siz=mi*vz+ci*uz;
        g[o]+=f0*mi*ux+f1*ci*vx+f2*six;g[o+1]+=f0*mi*uy+f1*ci*vy+f2*siy;g[o+2]+=f0*mi*uz+f1*ci*vz+f2*siz;
        if(hessian)for(let j=i;j<3;j++,pair++){
          const mj=memM[j],cj=memC[j],sjx=mj*vx+cj*ux,sjy=mj*vy+cj*uy,sjz=mj*vz+cj*uz;
          // A stretched direction's geometric stiffness is positive; a compressed one is dropped.
          const uu=k0*mi*mj,vv=k1*ci*cj,diagonal=tensionU*mi*mj+tensionV*ci*cj;
          addBlock(blocks,faceSlots[pair],
            uu*ux*ux+vv*vx*vx+k2*six*sjx+diagonal,uu*ux*uy+vv*vx*vy+k2*six*sjy,uu*ux*uz+vv*vx*vz+k2*six*sjz,
            uu*uy*ux+vv*vy*vx+k2*siy*sjx,uu*uy*uy+vv*vy*vy+k2*siy*sjy+diagonal,uu*uy*uz+vv*vy*vz+k2*siy*sjz,
            uu*uz*ux+vv*vz*vx+k2*siz*sjx,uu*uz*uy+vv*vz*vy+k2*siz*sjy,uu*uz*uz+vv*vz*vz+k2*siz*sjz+diagonal);
        }
      }
    }
    for(let j=0;j<hingeCount;j++){
      const a=3*ids[4*j],b=3*ids[4*j+1],c=3*ids[4*j+2],d=3*ids[4*j+3];
      const ax=x[a],ay=x[a+1],az=x[a+2],bx=x[b],by=x[b+1],bz=x[b+2],cx=x[c],cy=x[c+1],cz=x[c+2],dx=x[d],dy=x[d+1],dz=x[d+2];
      const ex=dx-cx,ey=dy-cy,ez=dz-cz,el=Math.sqrt(ex*ex+ey*ey+ez*ez);
      const ux=cx-ax,uy=cy-ay,uz=cz-az,vx=dx-ax,vy=dy-ay,vz=dz-az;
      const rx=dx-bx,ry=dy-by,rz=dz-bz,sx=cx-bx,sy=cy-by,sz=cz-bz;
      let n1x=uy*vz-uz*vy,n1y=uz*vx-ux*vz,n1z=ux*vy-uy*vx;
      let n2x=ry*sz-rz*sy,n2y=rz*sx-rx*sz,n2z=rx*sy-ry*sx;
      const sq1=n1x*n1x+n1y*n1y+n1z*n1z,sq2=n2x*n2x+n2y*n2y+n2z*n2z;
      if(el<1e-10||sq1<1e-18||sq2<1e-18)continue;
      const sin=((n1y*n2z-n1z*n2y)*ex+(n1z*n2x-n1x*n2z)*ey+(n1x*n2y-n1y*n2x)*ez)/el,cos=n1x*n2x+n1y*n2y+n1z*n2z;
      const sinTarget=hinges.targetSin[j],cosTarget=hinges.targetCos[j];
      const C=Math.atan2(sin*cosTarget-cos*sinTarget,cos*cosTarget+sin*sinTarget),k=inverse/hinges.alpha[j];
      energy+=k*C*C/2;
      if(!derivatives)continue;
      const force=k*C;
      n1x/=sq1;n1y/=sq1;n1z/=sq1;n2x/=sq2;n2y/=sq2;n2z/=sq2;
      const k2=(vx*ex+vy*ey+vz*ez)/el,l2=(rx*ex+ry*ey+rz*ez)/el,k3=-(ux*ex+uy*ey+uz*ez)/el,l3=-(sx*ex+sy*ey+sz*ez)/el;
      gradients[0]=-el*n1x;gradients[1]=-el*n1y;gradients[2]=-el*n1z;
      gradients[3]=-el*n2x;gradients[4]=-el*n2y;gradients[5]=-el*n2z;
      gradients[6]=k2*n1x+l2*n2x;gradients[7]=k2*n1y+l2*n2y;gradients[8]=k2*n1z+l2*n2z;
      gradients[9]=k3*n1x+l3*n2x;gradients[10]=k3*n1y+l3*n2y;gradients[11]=k3*n1z+l3*n2z;
      hingeIds[0]=a;hingeIds[1]=b;hingeIds[2]=c;hingeIds[3]=d;
      let pair=10*j;
      for(let p=0;p<4;p++){
        const o=hingeIds[p],px=gradients[3*p],py=gradients[3*p+1],pz=gradients[3*p+2];
        g[o]+=force*px;g[o+1]+=force*py;g[o+2]+=force*pz;
        if(hessian)for(let q=p;q<4;q++,pair++){
          const qx=k*gradients[3*q],qy=k*gradients[3*q+1],qz=k*gradients[3*q+2];
          addBlock(blocks,hingeSlots[pair],px*qx,px*qy,px*qz,py*qx,py*qy,py*qz,pz*qx,pz*qy,pz*qz);
        }
      }
    }
    return energy;
  }
  /** The sweeps' multiplier for a constraint with value C is −C/alpha at elastic equilibrium. */
  private multipliers(x:Float64Array,membranes:MembraneBatch,hinges:HingeBatch){
    const faces=membranes.faces,md=membranes.md,cd=membranes.cd,ids=hinges.ids;
    for(let t=0;t<faces.length;t+=3){
      const a=3*faces[t],b=3*faces[t+1],c=3*faces[t+2];
      const m0=md[t],m1=md[t+1],m2=md[t+2],c0=cd[t],c1=cd[t+1],c2=cd[t+2];
      const ux=m0*x[a]+m1*x[b]+m2*x[c],uy=m0*x[a+1]+m1*x[b+1]+m2*x[c+1],uz=m0*x[a+2]+m1*x[b+2]+m2*x[c+2];
      const vx=c0*x[a]+c1*x[b]+c2*x[c],vy=c0*x[a+1]+c1*x[b+1]+c2*x[c+1],vz=c0*x[a+2]+c1*x[b+2]+c2*x[c+2];
      membranes.lambda[t]=-(ux*ux+uy*uy+uz*uz-1)/2/membranes.alpha[t];
      membranes.lambda[t+1]=-(vx*vx+vy*vy+vz*vz-1)/2/membranes.alpha[t+1];
      membranes.lambda[t+2]=-(ux*vx+uy*vy+uz*vz)/membranes.alpha[t+2];
    }
    for(let j=0;j<hinges.lambda.length;j++){
      const a=3*ids[4*j],b=3*ids[4*j+1],c=3*ids[4*j+2],d=3*ids[4*j+3];
      const ex=x[d]-x[c],ey=x[d+1]-x[c+1],ez=x[d+2]-x[c+2],el=Math.sqrt(ex*ex+ey*ey+ez*ez);
      const ux=x[c]-x[a],uy=x[c+1]-x[a+1],uz=x[c+2]-x[a+2],vx=x[d]-x[a],vy=x[d+1]-x[a+1],vz=x[d+2]-x[a+2];
      const rx=x[d]-x[b],ry=x[d+1]-x[b+1],rz=x[d+2]-x[b+2],sx=x[c]-x[b],sy=x[c+1]-x[b+1],sz=x[c+2]-x[b+2];
      const n1x=uy*vz-uz*vy,n1y=uz*vx-ux*vz,n1z=ux*vy-uy*vx,n2x=ry*sz-rz*sy,n2y=rz*sx-rx*sz,n2z=rx*sy-ry*sx;
      if(el<1e-10||n1x*n1x+n1y*n1y+n1z*n1z<1e-18||n2x*n2x+n2y*n2y+n2z*n2z<1e-18){hinges.lambda[j]=0;continue;}
      const sin=((n1y*n2z-n1z*n2y)*ex+(n1z*n2x-n1x*n2z)*ey+(n1x*n2y-n1y*n2x)*ez)/el,cos=n1x*n2x+n1y*n2y+n1z*n2z;
      hinges.lambda[j]=-Math.atan2(sin*hinges.targetCos[j]-cos*hinges.targetSin[j],cos*hinges.targetCos[j]+sin*hinges.targetSin[j])/hinges.alpha[j];
    }
  }
  private factor(){
    const {ap,ai,ax,lp,parent,lnz,li,lx,d,y,pattern,flag,source,blocks}=this,size=3*this.n;
    for(let q=0;q<ax.length;q++)ax[q]=blocks[source[q]];
    for(let k=0;k<size;k++){
      y[k]=0;let top=size;flag[k]=k;lnz[k]=0;
      for(let p=ap[k];p<ap[k+1];p++){
        let i=ai[p];y[i]+=ax[p];let len=0;
        for(;flag[i]!==k;i=parent[i]){pattern[len++]=i;flag[i]=k;}
        while(len>0)pattern[--top]=pattern[--len];
      }
      let pivot=y[k];y[k]=0;
      for(;top<size;top++){
        const i=pattern[top],yi=y[i];y[i]=0;
        const end=lp[i]+lnz[i];
        for(let p=lp[i];p<end;p++)y[li[p]]-=lx[p]*yi;
        const l=yi/d[i];pivot-=l*yi;li[end]=k;lx[end]=l;lnz[i]++;
      }
      // Mass and Gauss–Newton stiffness are positive definite; guard against rounding only.
      if(!(pivot>0))return false;
      d[k]=pivot;
    }
    return true;
  }
  /** out = H⁻¹ g, in the caller's vertex order. */
  private backSolve(g:Float64Array,out:Float64Array){
    const {lp,li,lx,d,y,perm}=this,n=this.n,size=3*n;
    for(let i=0;i<n;i++){const p=3*perm[i],k=3*i;y[p]=g[k];y[p+1]=g[k+1];y[p+2]=g[k+2];}
    for(let j=0;j<size;j++){const yj=y[j];if(yj!==0)for(let p=lp[j];p<lp[j+1];p++)y[li[p]]-=lx[p]*yj;}
    for(let j=size-1;j>=0;j--){let s=y[j]/d[j];for(let p=lp[j];p<lp[j+1];p++)s-=lx[p]*y[li[p]];y[j]=s;}
    for(let i=0;i<n;i++){const p=3*perm[i],k=3*i;out[k]=y[p];out[k+1]=y[p+1];out[k+2]=y[p+2];}
    y.fill(0);
  }
}
/** Newton steps taken on one factorisation. */
const RENEW=3;
const memM=new Float64Array(3),memC=new Float64Array(3),memV=new Int32Array(3),hingeGradients=new Float64Array(12),hingeIdScratch=new Int32Array(4);
/** Add the 3×3 block d²/dx_i dx_j at a precomputed slot (transposed when the slot is negative). */
function addBlock(blocks:Float64Array,slot:number,xx:number,xy:number,xz:number,yx:number,yy:number,yz:number,zx:number,zy:number,zz:number){
  if(slot>=0){blocks[slot]+=xx;blocks[slot+1]+=xy;blocks[slot+2]+=xz;blocks[slot+3]+=yx;blocks[slot+4]+=yy;blocks[slot+5]+=yz;blocks[slot+6]+=zx;blocks[slot+7]+=zy;blocks[slot+8]+=zz;}
  else{const o=~slot;blocks[o]+=xx;blocks[o+1]+=yx;blocks[o+2]+=zx;blocks[o+3]+=xy;blocks[o+4]+=yy;blocks[o+5]+=zy;blocks[o+6]+=xz;blocks[o+7]+=yz;blocks[o+8]+=zz;}
}
/** Greedy minimum-degree ordering of the vertex graph; run once per mesh. */
function minimumDegree(n:number,pairs:[number,number][]){
  const adjacent=new Uint8Array(n*n),degree=new Int32Array(n),done=new Uint8Array(n),perm=new Int32Array(n);
  for(const [i,j] of pairs)if(i!==j&&!adjacent[i*n+j]){adjacent[i*n+j]=adjacent[j*n+i]=1;degree[i]++;degree[j]++;}
  const neighbours:number[]=[];
  for(let position=0;position<n;position++){
    let v=-1;for(let i=0;i<n;i++)if(!done[i]&&(v<0||degree[i]<degree[v]))v=i;
    done[v]=1;perm[v]=position;neighbours.length=0;
    for(let j=0;j<n;j++)if(!done[j]&&adjacent[v*n+j])neighbours.push(j);
    for(const a of neighbours){
      degree[a]--;
      for(const b of neighbours)if(a<b&&!adjacent[a*n+b]){adjacent[a*n+b]=adjacent[b*n+a]=1;degree[a]++;degree[b]++;}
    }
  }
  return perm;
}
