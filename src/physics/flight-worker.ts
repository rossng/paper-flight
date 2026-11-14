import { flightTransfers, type FlightRequest, type FlightResponse } from './flight-messages';

const port=globalThis as unknown as {
  postMessage(message:FlightResponse,transfer:Transferable[]):void;
  onmessage:(event:MessageEvent<FlightRequest>)=>void;
};
const send=(message:FlightResponse)=>port.postMessage(message,message.type==='state'?flightTransfers(message):[]);
// The flight solver loads asynchronously. Messages delivered before a handler is set are
// dropped, so hold them from the first synchronous turn until the loop exists.
const held:FlightRequest[]=[];
port.onmessage=event=>{held.push(event.data);};
try{
  const {FlightWorkerLoop}=await import('./flight-worker-loop');
  // MessageChannel avoids the timer clamp during bounded look-ahead. The loop
  // schedules nothing once it reaches its requested horizon.
  const channel=new MessageChannel(),tasks:(()=>void)[]=[];
  channel.port1.onmessage=()=>tasks.shift()?.();
  const loop=new FlightWorkerLoop(send,callback=>{tasks.push(callback);channel.port2.postMessage(null);});
  const handle=(message:FlightRequest)=>{try{loop.handle(message);}catch(error){send({type:'error',message:String(error)});}};
  port.onmessage=event=>handle(event.data);
  for(const message of held)handle(message);
}catch(error){send({type:'error',message:`The flight solver did not load: ${error}`});}
