import { FlightWorkerLoop } from './flight-worker-loop';
import { flightTransfers, type FlightRequest, type FlightResponse } from './flight-messages';

const port=globalThis as unknown as {
  postMessage(message:FlightResponse,transfer:Transferable[]):void;
  onmessage:(event:MessageEvent<FlightRequest>)=>void;
};
const send=(message:FlightResponse)=>port.postMessage(message,message.type==='state'?flightTransfers(message):[]);
// MessageChannel avoids the timer clamp during bounded look-ahead. The loop
// schedules nothing once it reaches its requested horizon.
const channel=new MessageChannel(),tasks:(()=>void)[]=[];
channel.port1.onmessage=()=>tasks.shift()?.();
const loop=new FlightWorkerLoop(send,callback=>{tasks.push(callback);channel.port2.postMessage(null);});
port.onmessage=event=>{try{loop.handle(event.data);}catch(error){send({type:'error',message:String(error)});}};
