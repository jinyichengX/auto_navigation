#!/usr/bin/env python3
"""Generate standalone.html from TypeScript source files."""
import os, re

BASE = os.path.dirname(os.path.abspath(__file__))

def read_ts(path):
    with open(path, 'r') as f:
        return f.read()

def strip_ts_types(code):
    """Convert TS object literal to JS object literal, removing types and export/import."""
    lines = code.split('\n')
    out = []
    for line in lines:
        s = line.strip()
        # Skip import
        if s.startswith('import ') or s.startswith('import{') or s.startswith('import {'):
            continue
        # Convert export const X: Type = { -> const X = {
        # Must preserve everything after the type annotation
        m = re.match(r'export const (\w+)(: [^=]+)? = (.+)', s)
        if m:
            out.append(f'const {m.group(1)} = {m.group(3)}')
            continue
        # Skip export type/interface lines
        if s.startswith('export type ') or s.startswith('export interface '):
            continue
        out.append(line)
    return '\n'.join(out)

# Read source files
comps_ts = read_ts(os.path.join(BASE, 'src', 'data', 'components.ts'))
steps_ts = read_ts(os.path.join(BASE, 'src', 'data', 'steps.ts'))

# Convert to plain JS
comps_js = strip_ts_types(comps_ts)
steps_js = strip_ts_types(steps_ts)

# HTML head
head = '''<!DOCTYPE html>
<html lang="zh-CN">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1.0">
<title>ROS 2 + Gazebo 架构可视化</title>
<script crossorigin src="https://unpkg.com/react@18/umd/react.production.min.js"></script>
<script crossorigin src="https://unpkg.com/react-dom@18/umd/react-dom.production.min.js"></script>
<script src="https://unpkg.com/@babel/standalone/babel.min.js"></script>
<link href="https://fonts.googleapis.com/css2?family=JetBrains+Mono:wght@400;500;700&family=Noto+Sans+SC:wght@400;500;700&display=swap" rel="stylesheet">
<style>
:root{--bg:#0f1419;--card:#1a2332;--bdr:#2d3a4a;--cy:#00d4ff;--or:#ff6b35;--pu:#a78bfa;--gr:#34d399;--rd:#f87171}
*{margin:0;padding:0;box-sizing:border-box}
html,body,#root{height:100%;overflow:hidden}
body{font-family:'Noto Sans SC',sans-serif;background:var(--bg);color:#e5e7eb}
::-webkit-scrollbar{width:5px}::-webkit-scrollbar-track{background:transparent}::-webkit-scrollbar-thumb{background:var(--bdr);border-radius:3px}
@keyframes pB{0%,100%{opacity:.6}50%{opacity:1}}
@keyframes sI{0%{transform:translateX(100%);opacity:0}100%{translateX(0);opacity:1}}
@keyframes fI{0%{opacity:0}100%{opacity:1}}
.ap{animation:pB 2s infinite}.as{animation:sI .3s}.af{animation:fI .3s}
</style>
</head>
<body><div id="root"></div>
<script type="text/babel">
const {createElement:e,useState,useEffect,useRef,useCallback,useMemo,useReducer,useContext,createContext,Fragment}=React;

// ===== DATA =====
'''

# After data, add store + components
logic = '''
let nid=0;function genId(){return"n"+ ++nid}
function cid(){return"c"+ ++nid}
function getSteps(mode){return mode==="real_robot"?realRobotSteps:mode==="multi_robot"?multiRobotSteps:simulationSteps}

const LAYOUT={xacro:{x:60,y:50},robot_description:{x:260,y:50},robot_state_publisher:{x:60,y:160},joint_state_publisher:{x:260,y:160},teleop:{x:60,y:350},controller_manager:{x:60,y:470},diff_drive_controller:{x:60,y:590},gz_sim:{x:750,y:30},gz_transport_layer:{x:750,y:130},physics_engine:{x:750,y:230},spawn_robot:{x:950,y:50},diff_drive_plugin:{x:950,y:310},gz_ros2_control_plugin:{x:950,y:420},ros_gz_bridge:{x:530,y:310},hardware_interface_gz:{x:950,y:520},hardware_interface_real:{x:600,y:520},real_motor_driver:{x:820,y:470},real_encoder:{x:820,y:580},real_imu:{x:820,y:200}};

function computeConnections(nodes){
  const conns=[];
  nodes.forEach(fn=>{const fd=componentDefs[fn.defId];if(!fd)return;
    fd.publishes.forEach(pub=>{nodes.forEach(tn=>{if(fn.iid===tn.iid)return;const td=componentDefs[tn.defId];if(!td)return;
      td.subscribes.forEach(sub=>{if(pub.topic===sub.topic){
        let tp="state";if(pub.topic==="/cmd_vel")tp="cmd";else if(pub.topic==="/tf"||pub.topic==="/tf_static")tp="tf";
        let tr="ros2_dds";if(pub.transport!==sub.transport)tr="cross";else if(pub.transport==="gz_transport")tr="gz_transport";
        if(fd.id==="ros_gz_bridge"||td.id==="ros_gz_bridge")tp="bridge";
        const exists=conns.find(c=>c.fn===fn.iid&&c.tn===tn.iid&&c.topic===pub.topic);
        if(!exists)conns.push({id:cid(),fn:fn.iid,ft:pub.topic,tn:tn.iid,tt:sub.topic,topic:pub.topic,type:tp,transport:tr});
      }});
    })});
  });
  return conns;
}

const initState={mode:"simulation",guideMode:"guided",currentStep:0,nodes:[],connections:[],selectedNodeId:null};

function reducer(state,action){
  switch(action.type){
    case"SET_MODE":{return{...state,mode:action.mode,currentStep:0,nodes:[],connections:[],selectedNodeId:null}}
    case"SET_GUIDE":return{...state,guideMode:action.mode}
    case"ADD_NODE":{
      const def=componentDefs[action.defId];if(!def)return state;
      const pos=LAYOUT[action.defId]||{x:50+Math.random()*400,y:100+Math.random()*400};
      const node={iid:genId(),defId:action.defId,x:action.x||pos.x,y:action.y||pos.y,robotId:action.robotId};
      const nodes=[...state.nodes,node];
      return{...state,nodes,connections:computeConnections(nodes)};
    }
    case"REMOVE_NODE":{
      const nodes=state.nodes.filter(n=>n.iid!==action.iid);
      return{...state,nodes,connections:computeConnections(nodes),selectedNodeId:state.selectedNodeId===action.iid?null:state.selectedNodeId};
    }
    case"MOVE_NODE":{
      const nodes=state.nodes.map(n=>n.iid===action.iid?{...n,x:action.x,y:action.y}:n);
      return{...state,nodes,connections:computeConnections(nodes)};
    }
    case"SELECT_NODE":return{...state,selectedNodeId:action.iid||null}
    case"NEXT_STEP":{
      const steps=getSteps(state.mode);
      const si=state.currentStep+1;if(si>=steps.length)return state;
      const step=steps[si],removeIds=new Set(step.removeNodes||[]);
      let nodes=state.nodes.filter(n=>!removeIds.has(n.defId));
      const existingIds=new Set(nodes.map(n=>n.defId));
      step.addNodes.forEach(defId=>{if(!existingIds.has(defId)){const pos=LAYOUT[defId]||{x:50,y:200};nodes.push({iid:genId(),defId,x:pos.x,y:pos.y});}});
      return{...state,currentStep:si,nodes,connections:computeConnections(nodes)};
    }
    case"PREV_STEP":{
      if(state.currentStep<=0)return state;
      const steps=getSteps(state.mode),curr=steps[state.currentStep];
      const reAddIds=new Set(curr.removeNodes||[]);
      let nodes=[...state.nodes];reAddIds.forEach(defId=>{if(!nodes.find(n=>n.defId===defId)){const pos=LAYOUT[defId]||{x:50,y:200};nodes.push({iid:genId(),defId,x:pos.x,y:pos.y});}});
      const removeIds=new Set(curr.addNodes.filter(id=>!reAddIds.has(id)));
      nodes=nodes.filter(n=>!removeIds.has(n.defId));
      return{...state,currentStep:state.currentStep-1,nodes,connections:computeConnections(nodes)};
    }
    case"JUMP_STEP":{
      const steps=getSteps(state.mode);if(action.step<0||action.step>=steps.length)return state;
      let cur=new Set(),map={};
      for(let i=0;i<=action.step;i++){const s=steps[i];(s.removeNodes||[]).forEach(r=>cur.delete(r));s.addNodes.forEach(defId=>{if(!cur.has(defId)){cur.add(defId);const pos=LAYOUT[defId]||{x:50,y:200};map[defId]={iid:genId(),defId,x:pos.x,y:pos.y};}});}
      const nodes=Object.values(map).filter(n=>cur.has(n.defId));
      return{...state,currentStep:action.step,nodes,connections:computeConnections(nodes)};
    }
    default:return state;
  }
}

const StoreCtx=createContext(null);
function StoreProvider({children}){
  const[state,dispatch]=useReducer(reducer,initState);
  const actions=useMemo(()=>({
    setMode:m=>dispatch({type:"SET_MODE",mode:m}),
    setGuideMode:m=>dispatch({type:"SET_GUIDE",mode:m}),
    addNode:(defId,x,y,robotId)=>dispatch({type:"ADD_NODE",defId,x,y,robotId}),
    removeNode:iid=>dispatch({type:"REMOVE_NODE",iid}),
    moveNode:(iid,x,y)=>dispatch({type:"MOVE_NODE",iid,x,y}),
    selectNode:iid=>dispatch({type:"SELECT_NODE",iid}),
    nextStep:()=>dispatch({type:"NEXT_STEP"}),
    prevStep:()=>dispatch({type:"PREV_STEP"}),
    jumpToStep:s=>dispatch({type:"JUMP_STEP",step:s}),
  }),[]);
  return e(StoreCtx.Provider,{value:{...state,...actions}},children);
}
function useStore(){return useContext(StoreCtx)}
'''

# React components
components_js = '''
function Header(){
  const s=useStore(),modes=[{id:"simulation",label:"仿真模式"},{id:"real_robot",label:"真机模式"},{id:"compare",label:"仿真 vs 真机"},{id:"multi_robot",label:"多机器人"}];
  return e("header",{className:"h-12 flex items-center px-4 shrink-0",style:{background:"var(--card)",borderBottom:"1px solid var(--bdr)"}},
    e("div",{style:{display:"flex",alignItems:"center",gap:8,marginRight:24}},
      e("span",{style:{color:"var(--cy)",fontFamily:"JetBrains Mono",fontSize:14,fontWeight:700,letterSpacing:1}},"ROS2\u00d7GZ"),
      e("span",{style:{color:"#6b7280",fontSize:12}},"架构可视化")
    ),
    e("div",{style:{display:"flex",gap:4}},modes.map(m=>
      e("button",{key:m.id,onClick:()=>s.setMode(m.id),
        style:{padding:"2px 10px",borderRadius:6,fontSize:12,fontWeight:500,border:`1px solid ${s.mode===m.id?"rgba(0,212,255,.4)":"transparent"}`,
        background:s.mode===m.id?"rgba(0,212,255,.15)":"transparent",color:s.mode===m.id?"var(--cy)":"#9ca3af",cursor:"pointer",transition:"all .2s"},
        onMouseEnter:()=>{},onMouseLeave:(){}
      },m.label)
    )),
    e("div",{style:{marginLeft:"auto",display:"flex",alignItems:"center",gap:8}},
      e("span",{style:{color:"#6b7280",fontSize:12}},"模式："),
      e("button",{onClick:()=>s.setGuideMode("guided"),style:{padding:"2px 10px",borderRadius:6,fontSize:12,fontWeight:500,border:`1px solid ${s.guideMode==="guided"?"rgba(167,139,250,.4)":"transparent"}`,background:s.guideMode==="guided"?"rgba(167,139,250,.15)":"transparent",color:s.guideMode==="guided"?"var(--pu)":"#9ca3af",cursor:"pointer"}},"逐步引导"),
      e("button",{onClick:()=>s.setGuideMode("free"),style:{padding:"2px 10px",borderRadius:6,fontSize:12,fontWeight:500,border:`1px solid ${s.guideMode==="free"?"rgba(167,139,250,.4)":"transparent"}`,background:s.guideMode==="free"?"rgba(167,139,250,.15)":"transparent",color:s.guideMode==="free"?"var(--pu)":"#9ca3af",cursor:"pointer"}},"自由探索")
    )
  );
}

function Sidebar(){
  const s=useStore();
  const cats=["ros2_core","gazebo","bridge","ros2_control","hardware","concept"];
  const grouped={};cats.forEach(c=>grouped[c]=[]);
  Object.entries(componentDefs).forEach(([id,def])=>grouped[def.category].push(id));
  const hds=e=>{e.dataTransfer.setData("text/plain",e.currentTarget.dataset.defid);e.dataTransfer.effectAllowed="copy";};

  if(s.guideMode==="guided"){
    return e("aside",{className:"w-56 shrink-0 overflow-y-auto p-3",style:{background:"var(--card)",borderRight:"1px solid var(--bdr)"}},
      e("div",{style:{color:"#6b7280",fontSize:12,textTransform:"uppercase",letterSpacing:1,marginBottom:8,fontFamily:"JetBrains Mono"}},"逐步引导模式"),
      e("div",{style:{color:"#9ca3af",fontSize:12,lineHeight:1.6}},"组件将按预设顺序自动添加到画布。",e("br"),e("br"),"用底部的「下一步」按钮推进。",e("br"),e("br"),"也可切换到「自由探索」自行拖拽。")
    );
  }
  return e("aside",{className:"w-56 shrink-0 overflow-y-auto",style:{background:"var(--card)",borderRight:"1px solid var(--bdr)"}},
    e("div",{style:{padding:12}},
      e("div",{style:{color:"#6b7280",fontSize:12,textTransform:"uppercase",letterSpacing:1,marginBottom:8,fontFamily:"JetBrains Mono"}},"组件库"),
      e("div",{style:{color:"#6b7280",fontSize:12,marginBottom:12}},"拖拽到画布中"),
      cats.map(cat=>{const info=categoryInfo[cat],items=grouped[cat]||[];if(!items.length)return null;
        return e("div",{key:cat,style:{marginBottom:12}},
          e("div",{style:{display:"flex",alignItems:"center",gap:6,marginBottom:6}},
            e("span",{style:{color:info.color,fontSize:12}},info.icon),
            e("span",{style:{color:info.color,fontSize:12,fontWeight:500}},info.label)
          ),
          e("div",{className:"space-y-1"},items.map(defId=>{const def=componentDefs[defId];
            return e("div",{key:defId,draggable:true,"data-defid":defId,onDragStart:hds,
              style:{padding:"4px 8px",borderRadius:4,fontSize:11,cursor:"grab",border:`1px solid ${def.borderColor.replace("border-","").replace("/60","")}66`,background:def.color.replace("bg-","").replace("/60",""),overflow:"hidden",whiteSpace:"nowrap",textOverflow:"ellipsis",marginBottom:4},title:def.description.replace(/\\n/g," ").substring(0,200)},def.name);
          }))
        );
      })
    )
  );
}

function DetailPanel(){
  const s=useStore();
  if(!s.selectedNodeId){
    return e("aside",{className:"w-72 shrink-0 overflow-y-auto p-4",style:{background:"var(--card)",borderLeft:"1px solid var(--bdr)"}},
      e("div",{style:{color:"#6b7280",fontSize:12,textTransform:"uppercase",letterSpacing:1,marginBottom:12,fontFamily:"JetBrains Mono"}},"组件详情"),
      e("div",{style:{color:"#6b7280",fontSize:12,lineHeight:1.6}},"点击画布中的组件查看详细信息。")
    );
  }
  const node=s.nodes.find(n=>n.iid===s.selectedNodeId);if(!node)return null;
  const def=componentDefs[node.defId];if(!def)return null;
  const bl=def.belongsTo==="ros2_dds"?"ROS 2 DDS":def.belongsTo==="gz_transport"?"Gazebo Transport":def.belongsTo==="both"?"跨边界":def.belongsTo==="real_hardware"?"真实硬件":"概念";
  const blc=def.belongsTo==="ros2_dds"?"rgba(6,78,59,.4)":def.belongsTo==="gz_transport"?"rgba(22,78,99,.35)":def.belongsTo==="both"?"rgba(124,45,18,.35)":def.belongsTo==="real_hardware"?"rgba(127,29,29,.35)":"rgba(51,65,85,.4)";
  const blt=def.belongsTo==="ros2_dds"?"var(--gr)":def.belongsTo==="gz_transport"?"var(--cy)":def.belongsTo==="both"?"var(--or)":def.belongsTo==="real_hardware"?"var(--rd)":"#94a3b8";

  return e("aside",{className:"w-72 shrink-0 overflow-y-auto as",style:{background:"var(--card)",borderLeft:"1px solid var(--bdr)"}},
    e("div",{style:{padding:16}},
      e("div",{style:{display:"flex",alignItems:"center",justifyContent:"space-between",marginBottom:12}},
        e("span",{style:{color:"#6b7280",fontSize:12,textTransform:"uppercase",letterSpacing:1,fontFamily:"JetBrains Mono"}},"组件详情"),
        e("button",{onClick:()=>s.selectNode(null),style:{color:"#6b7280",fontSize:14,background:"none",border:"none",cursor:"pointer"}},"\u2715")
      ),
      e("div",{style:{borderRadius:8,border:`1px solid ${def.borderColor.replace("border-","").replace("/60","")}66`,background:def.color.replace("bg-","").replace("/60",""),padding:12,marginBottom:12}},
        e("div",{style:{fontSize:13,fontWeight:"bold",color:"#e5e7eb",whiteSpace:"pre-line"}},def.name),
        e("div",{style:{fontSize:11,color:"#9ca3af",marginTop:2,fontFamily:"JetBrains Mono"}},def.pkg)
      ),
      (def.exec&&def.exec!=="-")?e("div",{style:{marginBottom:12}},
        e("div",{style:{fontSize:11,color:"var(--cy)",fontFamily:"JetBrains Mono",marginBottom:4}},"可执行文件"),
        e("code",{style:{fontSize:11,background:"rgba(0,0,0,.3)",borderRadius:4,padding:"2px 8px",display:"block",color:"#d1d5db"}},def.exec)
      ):null,
      def.subscribes.length>0?e("div",{style:{marginBottom:12}},
        e("div",{style:{fontSize:11,color:"var(--gr)",fontFamily:"JetBrains Mono",marginBottom:4}},"订阅 (Subscribe)"),
        def.subscribes.map((s,i)=>e("div",{key:i,style:{fontSize:11,background:"rgba(0,0,0,.3)",borderRadius:4,padding:"2px 8px",marginBottom:3}},
          e("span",{style:{color:"#d1d5db",fontFamily:"JetBrains Mono"}},s.topic)," ",
          e("span",{style:{color:"#6b7280"}},"("+s.type+")")," ",
          e("span",{style:{color:"#4b5563",fontSize:10}},"["+s.transport+"]")
        ))
      ):null,
      def.publishes.length>0?e("div",{style:{marginBottom:12}},
        e("div",{style:{fontSize:11,color:"var(--or)",fontFamily:"JetBrains Mono",marginBottom:4}},"发布 (Publish)"),
        def.publishes.map((p,i)=>e("div",{key:i,style:{fontSize:11,background:"rgba(0,0,0,.3)",borderRadius:4,padding:"2px 8px",marginBottom:3}},
          e("span",{style:{color:"#d1d5db",fontFamily:"JetBrains Mono"}},p.topic)," ",
          e("span",{style:{color:"#6b7280"}},"("+p.type+")")," ",
          e("span",{style:{color:"#4b5563",fontSize:10}},"["+p.transport+"]")
        ))
      ):null,
      e("div",{style:{marginBottom:12}},
        e("div",{style:{fontSize:11,color:"var(--pu)",fontFamily:"JetBrains Mono",marginBottom:4}},"所属域"),
        e("span",{style:{fontSize:11,padding:"1px 8px",borderRadius:4,background:blc,color:blt}},bl)
      ),
      e("div",null,
        e("div",{style:{fontSize:11,color:"#9ca3af",fontFamily:"JetBrains Mono",marginBottom:4}},"详细解释"),
        e("div",{style:{fontSize:11,color:"#d1d5db",lineHeight:1.7,whiteSpace:"pre-line"}},def.description)
      )
    )
  );
}

function ConnectionLine({conn}){
  const s=useStore();
  const fn=s.nodes.find(n=>n.iid===conn.fn),tn=s.nodes.find(n=>n.iid===conn.tn);
  if(!fn||!tn)return null;
  const fdef=componentDefs[fn.defId],tdef=componentDefs[tn.defId];
  const fw=fdef?fdef.w:170,fh=fdef?fdef.h:65;
  const tw=tdef?tdef.w:170,th=tdef?tdef.h:65;
  const fx=fn.x+fw,fy=fn.y+fh/2;
  const tx=tn.x,ty=tn.y+th/2;
  const dx=tx-fx,cp=Math.min(Math.abs(dx)*.4,100);
  let p;if(dx>30)p=`M ${fx} ${fy} C ${fx+cp} ${fy}, ${tx-cp} ${ty}, ${tx} ${ty}`;else p=`M ${fx} ${fy} L ${tx} ${ty}`;
  const colors={ros2_dds:"var(--gr)",gz_transport:"var(--cy)",cross:"var(--or)"};
  const markers={ros2_dds:"url(#ar-ros2)",gz_transport:"url(#ar-gz)",cross:"url(#ar-cross)"};
  const c=colors[conn.transport]||"#94a3b8";
  const isC=conn.transport==="cross";
  return e("g",null,
    e("path",{d:p,fill:"none",stroke:c,strokeWidth:isC?2:1.5,strokeDasharray:isC?"6,3":"none",markerEnd:markers[conn.transport]||"url(#ar-def)",opacity:isC?.8:.5,className:isC?"ap":""}),
    e("text",{x:(fx+tx)/2,y:(fy+ty)/2-8,textAnchor:"middle",fill:c,fontSize:9,fontFamily:"JetBrains Mono, monospace",opacity:.8},conn.topic)
  );
}

function CanvasNode({node,scale}){
  const s=useStore();
  const def=componentDefs[node.defId];if(!def)return null;
  const isSel=s.selectedNodeId===node.iid;
  const[drag,setDrag]=useState(false);
  const[ctx,setCtx]=useState(false);
  const dr=useRef(null);

  const onMD=useCallback(ev=>{ev.stopPropagation();s.selectNode(node.iid);setDrag(true);
    dr.current={sx:ev.clientX,sy:ev.clientY,nx:node.x,ny:node.y};
    const mm=e2=>{if(!dr.current)return;const dx=(e2.clientX-dr.current.sx)/scale,dy=(e2.clientY-dr.current.sy)/scale;s.moveNode(node.iid,dr.current.nx+dx,dr.current.ny+dy);};
    const mu=()=>{setDrag(false);dr.current=null;document.removeEventListener("mousemove",mm);document.removeEventListener("mouseup",mu);};
    document.addEventListener("mousemove",mm);document.addEventListener("mouseup",mu);
  },[node.iid,node.x,node.y,scale,s]);

  const onCtx=useCallback(ev=>{ev.preventDefault();ev.stopPropagation();setCtx(true);s.selectNode(node.iid);
    setTimeout(()=>document.addEventListener("click",()=>setCtx(false),{once:true}),0);
  },[node.iid,s]);

  const isC=def.category==="concept",w=def.w,h=def.h;

  return e("div",{"data-node":"true",
    style:{position:"absolute",left:node.x,top:node.y,width:w,minHeight:h,borderRadius:8,
      border:`1px solid ${def.borderColor.replace("border-","").replace("/60","")}66`,
      background:def.color.replace("bg-","").replace("/60",""),
      cursor:drag?"grabbing":"grab",opacity:drag?.9:1,
      boxShadow:isSel?"0 0 15px rgba(0,212,255,.2)":"none",
      borderStyle:isC?"dashed":"solid",zIndex:isSel?10:0},
    onMouseDown:onMD,onContextMenu:onCtx,onClick:ev=>ev.stopPropagation()},
    e("div",{style:{padding:8}},
      e("div",{style:{fontSize:12,fontWeight:500,color:"#e5e7eb",whiteSpace:"pre-line",lineHeight:1.3}},def.name),
      (def.pkg&&def.pkg!=="-")?e("div",{style:{fontSize:10,color:"#6b7280",fontFamily:"JetBrains Mono",marginTop:2,overflow:"hidden",textOverflow:"ellipsis"}},def.pkg):null
    ),
    ctx?e("div",{style:{position:"absolute",top:"100%",left:0,marginTop:4,background:"var(--card)",border:"1px solid var(--bdr)",borderRadius:8,padding:"4px 0",zIndex:50,minWidth:100,boxShadow:"0 4px 20px rgba(0,0,0,.5)"},onClick:ev=>ev.stopPropagation()},
      e("button",{onClick:ev=>{ev.stopPropagation();s.removeNode(node.iid);setCtx(false);},
        style:{width:"100%",textAlign:"left",padding:"6px 12px",fontSize:12,color:"#f87171",background:"none",border:"none",cursor:"pointer",display:"flex",alignItems:"center",gap:8}},
        e("span",null,"\u2715"),"删除此节点")
    ):null
  );
}

function StepGuide(){
  const s=useStore();
  const inited=useRef(false);
  const steps=getSteps(s.mode);

  useEffect(()=>{inited.current=false},[s.mode]);
  useEffect(()=>{if(s.guideMode==="guided"&&s.nodes.length===0&&!inited.current){inited.current=true;s.jumpToStep(0)}},[s.guideMode,s.nodes.length,s]);

  if(s.guideMode!=="guided")return null;
  if(s.currentStep<0||s.currentStep>=steps.length)return null;

  const step=steps[s.currentStep],isF=s.currentStep===0,isL=s.currentStep===steps.length-1;

  return e("div",{className:"af",style:{position:"absolute",bottom:40,left:"50%",transform:"translateX(-50%)",zIndex:30}},
    e("div",{style:{background:"var(--card)",border:"1px solid var(--bdr)",borderRadius:12,padding:16,maxWidth:480,boxShadow:"0 4px 30px rgba(0,0,0,.6)"}},
      e("div",{style:{display:"flex",alignItems:"center",gap:8,marginBottom:12}},
        e("div",{style:{flex:1,height:4,background:"var(--bg)",borderRadius:2,overflow:"hidden"}},
          e("div",{style:{height:"100%",background:"var(--cy)",borderRadius:2,width:((s.currentStep+1)/steps.length*100)+"%",transition:"width .5s"}})
        ),
        e("span",{style:{color:"#6b7280",fontSize:12,fontFamily:"JetBrains Mono"}},(s.currentStep+1)+"/"+steps.length)
      ),
      e("h3",{style:{fontSize:14,fontWeight:"bold",color:"#e5e7eb",marginBottom:8}},"步骤 "+step.step+"："+step.title),
      e("p",{style:{fontSize:12,color:"#d1d5db",lineHeight:1.6,whiteSpace:"pre-line",maxHeight:180,overflowY:"auto"}},step.desc),
      e("div",{style:{display:"flex",alignItems:"center",justifyContent:"space-between",marginTop:12,paddingTop:12,borderTop:"1px solid var(--bdr)"}},
        e("div",{style:{display:"flex",gap:4}},
          e("button",{onClick:s.prevStep,disabled:isF,
            style:{padding:"4px 12px",borderRadius:6,fontSize:12,fontWeight:500,border:isF?"none":"1px solid var(--bdr)",background:"transparent",color:isF?"#4b5563":"#d1d5db",cursor:isF?"not-allowed":"pointer"}},"\u25c0 上一步"),
          e("button",{onClick:s.nextStep,disabled:isL,
            style:{padding:"4px 12px",borderRadius:6,fontSize:12,fontWeight:500,border:`1px solid ${isL?"transparent":"rgba(0,212,255,.4)"}`,background:isL?"transparent":"rgba(0,212,255,.15)",color:isL?"#4b5563":"var(--cy)",cursor:isL?"not-allowed":"pointer"}},"下一步 \u25b6")
        ),
        e("div",{style:{display:"flex",gap:4}},steps.map((_,i)=>
          e("button",{key:i,onClick:()=>s.jumpToStep(i),
            style:{width:i===s.currentStep?10:8,height:i===s.currentStep?10:8,borderRadius:"50%",border:"none",
              background:i===s.currentStep?"var(--cy)":i<s.currentStep?"rgba(0,212,255,.3)":"#4b5563",
              cursor:"pointer",transition:"all .2s",transform:i===s.currentStep?"scale(1.2)":"scale(1)"}})
        ))
      )
    )
  );
}

function StatusBar(){
  const s=useStore(),ml={simulation:"仿真模式",real_robot:"真机模式",compare:"仿真 vs 真机",multi_robot:"多机器人"};
  return e("footer",{style:{height:26,background:"var(--card)",borderTop:"1px solid var(--bdr)",display:"flex",alignItems:"center",padding:"0 16px",fontSize:11,color:"#6b7280",fontFamily:"JetBrains Mono"}},
    e("span",{style:{marginRight:14}},"模式: ",e("span",{style:{color:"var(--cy)"}},ml[s.mode])),
    e("span",{style:{marginRight:14}},"引导: ",e("span",{style:{color:s.guideMode==="guided"?"var(--pu)":"#9ca3af"}},s.guideMode==="guided"?"逐步引导":"自由探索")),
    s.guideMode==="guided"?e("span",{style:{marginRight:14}},"步骤: ",e("span",{style:{color:"#d1d5db"}},s.currentStep)):null,
    e("span",{style:{marginRight:14}},"节点: ",e("span",{style:{color:"#d1d5db"}},s.nodes.length)),
    e("span",{style:{marginRight:14}},"连线: ",e("span",{style:{color:"#d1d5db"}},s.connections.length)),
    e("span",{style:{marginLeft:"auto",color:"#4b5563"}},"滚轮缩放 \u00b7 拖拽节点 \u00b7 右键删除 \u00b7 点击详情 \u00b7 \u2192/\u2190 切换步骤")
  );
}

function Canvas(){
  const s=useStore();
  const ref=useRef(null);
  const[scale,setScale]=useState(.9);
  const[offset,setOffset]=useState({x:0,y:30});
  const[panning,setPanning]=useState(false);
  const pr=useRef({sx:0,sy:0,ox:0,oy:0});
  const sRef=useRef(scale),_oRef=useRef(offset);
  sRef.current=scale;_oRef.current=offset;

  const onWheel=useCallback(ev=>{ev.preventDefault();const d=ev.deltaY>0?-.05:.05;setScale(v=>Math.max(.2,Math.min(2,v+d)))},[]);

  const onMD=useCallback(ev=>{if(ev.target.closest("[data-node]"))return;setPanning(true);
    setOffset(prev=>{pr.current={sx:ev.clientX,sy:ev.clientY,ox:prev.x,oy:prev.y};return prev;});
    s.selectNode(null);
    const mm=e2=>{const dx=e2.clientX-pr.current.sx,dy=e2.clientY-pr.current.sy;setOffset({x:pr.current.ox+dx,y:pr.current.oy+dy});};
    const mu=()=>{setPanning(false);document.removeEventListener("mousemove",mm);document.removeEventListener("mouseup",mu);};
    document.addEventListener("mousemove",mm);document.addEventListener("mouseup",mu);
  },[s.selectNode]);

  const onDrop=useCallback(ev=>{ev.preventDefault();
    const defId=ev.dataTransfer.getData("text/plain");if(!defId||!componentDefs[defId])return;
    const r=ref.current?.getBoundingClientRect();if(!r)return;
    const ss=sRef.current,oo=_oRef.current;
    s.addNode(defId,(ev.clientX-r.left-oo.x)/ss,(ev.clientY-r.top-oo.y)/ss);
  },[s.addNode]);

  const onDO=useCallback(ev=>{ev.preventDefault();ev.dataTransfer.dropEffect="copy"},[]);

  const bx=530;
  const ros2=s.nodes.filter(n=>{const d=componentDefs[n.defId];return d&&(d.belongsTo==="ros2_dds"||d.id==="robot_description")});
  const gz=s.nodes.filter(n=>{const d=componentDefs[n.defId];return d&&(d.belongsTo==="gz_transport"||d.belongsTo==="both")});
  const sB=ros2.length>0||gz.length>0;

  // Keyboard
  useEffect(()=>{const h=e2=>{if(s.guideMode==="guided"){if(e2.key==="ArrowRight"||e2.key===" "){e2.preventDefault();s.nextStep()}if(e2.key==="ArrowLeft"){e2.preventDefault();s.prevStep()}}};window.addEventListener("keydown",h);return()=>window.removeEventListener("keydown",h)},[s.guideMode,s.nextStep,s.prevStep]);

  return e("div",{ref,className:"flex-1 relative overflow-hidden",
    style:{background:"radial-gradient(circle at 50% 50%, #141e2b 0%, var(--bg) 100%)",backgroundImage:"radial-gradient(circle at 50% 50%, #141e2b 0%, var(--bg) 100%),linear-gradient(rgba(45,58,74,.12) 1px,transparent 1px),linear-gradient(90deg,rgba(45,58,74,.12) 1px,transparent 1px)",backgroundSize:"100% 100%, 30px 30px, 30px 30px",cursor:panning?"grabbing":"grab"},
    onWheel,onMouseDown:onMD,onDrop,onDragOver:onDO},
    // Inner layer
    e("div",{style:{transform:`scale(${scale}) translate(${offset.x/scale}px, ${offset.y/scale}px)`,transformOrigin:"0 0",position:"absolute",top:0,left:0,width:"100%",height:"100%"}},
      sB?e("div",{style:{position:"absolute",top:0,bottom:0,left:bx,borderLeft:"2px dashed rgba(75,85,99,.35)",height:"300vh"}},
        e("div",{style:{position:"absolute",left:-68,top:10,fontSize:10,color:"rgba(52,211,153,.5)",fontFamily:"JetBrains Mono",letterSpacing:2}},"ROS 2 DDS"),
        e("div",{style:{position:"absolute",left:6,top:10,fontSize:10,color:"rgba(0,212,255,.5)",fontFamily:"JetBrains Mono",letterSpacing:2}},"Gazebo Transport")
      ):null,
      e("svg",{className:"absolute top-0 left-0 pointer-events-none",style:{width:"100%",height:"100%",minWidth:2000,minHeight:1500}},
        e("defs",null,
          e("marker",{id:"ar-ros2",viewBox:"0 0 10 8",refX:10,refY:4,markerWidth:8,markerHeight:6,orient:"auto"},e("path",{d:"M 0 0 L 10 4 L 0 8 z",fill:"var(--gr)"})),
          e("marker",{id:"ar-gz",viewBox:"0 0 10 8",refX:10,refY:4,markerWidth:8,markerHeight:6,orient:"auto"},e("path",{d:"M 0 0 L 10 4 L 0 8 z",fill:"var(--cy)"})),
          e("marker",{id:"ar-cross",viewBox:"0 0 10 8",refX:10,refY:4,markerWidth:8,markerHeight:6,orient:"auto"},e("path",{d:"M 0 0 L 10 4 L 0 8 z",fill:"var(--or)"})),
          e("marker",{id:"ar-def",viewBox:"0 0 10 8",refX:10,refY:4,markerWidth:8,markerHeight:6,orient:"auto"},e("path",{d:"M 0 0 L 10 4 L 0 8 z",fill:"#94a3b8"}))
        ),
        s.connections.map(conn=>e(ConnectionLine,{key:conn.id,conn}))
      ),
      s.nodes.map(node=>e(CanvasNode,{key:node.iid,node,scale})),
      (s.nodes.length===0)?e("div",{style:{position:"absolute",top:"50%",left:"50%",transform:"translate(-50%,-50%)",textAlign:"center"}},
        e("div",{style:{color:"#4b5563",fontSize:14,fontFamily:"JetBrains Mono",marginBottom:6}},s.guideMode==="guided"?"点击「下一步」开始逐步引导":"从左侧拖拽组件到此处"),
        e("div",{style:{color:"#374151",fontSize:11}},"滚轮缩放 \u00b7 空白处拖拽平移")
      ):null
    ),
    // Zoom controls
    e("div",{style:{position:"absolute",bottom:8,right:8,display:"flex",gap:4,zIndex:10}},
      e("button",{onClick:()=>setScale(v=>Math.min(2,v+.1)),style:{width:26,height:26,borderRadius:4,background:"var(--card)",border:"1px solid var(--bdr)",color:"#9ca3af",fontSize:14,cursor:"pointer",display:"flex",alignItems:"center",justifyContent:"center"}},"+"),
      e("button",{onClick:()=>setScale(v=>Math.max(.2,v-.1)),style:{width:26,height:26,borderRadius:4,background:"var(--card)",border:"1px solid var(--bdr)",color:"#9ca3af",fontSize:14,cursor:"pointer",display:"flex",alignItems:"center",justifyContent:"center"}},"\u2212"),
      e("span",{style:{fontSize:10,color:"#6b7280",fontFamily:"JetBrains Mono",padding:"0 4px",display:"flex",alignItems:"center"}},Math.round(scale*100)+"%")
    ),
    // Legend
    e("div",{style:{position:"absolute",top:8,right:8,background:"rgba(26,35,50,.9)",border:"1px solid var(--bdr)",borderRadius:6,padding:"6px 8px",zIndex:10,fontSize:10,fontFamily:"JetBrains Mono"}},
      e("div",{style:{display:"flex",alignItems:"center",gap:6,marginBottom:3}},e("div",{style:{width:10,height:2,borderRadius:1,background:"var(--gr)"}}),e("span",{style:{color:"#9ca3af"}},"ROS 2 DDS")),
      e("div",{style:{display:"flex",alignItems:"center",gap:6,marginBottom:3}},e("div",{style:{width:10,height:2,borderRadius:1,background:"var(--cy)"}}),e("span",{style:{color:"#9ca3af"}},"Gazebo Transport")),
      e("div",{style:{display:"flex",alignItems:"center",gap:6}},e("div",{style:{width:10,height:2,borderRadius:1,background:"var(--or)"}}),e("span",{style:{color:"#9ca3af"}},"跨边界桥接"))
    )
  );
}

function App(){
  const s=useStore();

  // Keyboard shortcut
  useEffect(()=>{const h=e2=>{if(s.guideMode==="guided"){if(e2.key==="ArrowRight"||e2.key===" "){e2.preventDefault();s.nextStep()}if(e2.key==="ArrowLeft"){e2.preventDefault();s.prevStep()}}};window.addEventListener("keydown",h);return()=>window.removeEventListener("keydown",h)},[s.guideMode,s]);

  if(s.mode==="compare"){
    return e("div",{style:{height:"100%",display:"flex",flexDirection:"column"}},
      e(Header),e("div",{style:{flex:1,display:"flex",overflow:"hidden"}},
        e(Sidebar),
        e(Canvas),
        e("div",{className:"w-80 shrink-0 overflow-y-auto p-4",style:{background:"var(--card)",borderLeft:"1px solid var(--bdr)"}},
          e("div",{style:{color:"#6b7280",fontSize:12,textTransform:"uppercase",letterSpacing:1,marginBottom:14,fontFamily:"JetBrains Mono"}},"仿真 vs 真机对比"),
          e("div",{style:{marginBottom:14}},
            e("div",{style:{color:"var(--cy)",fontFamily:"JetBrains Mono",fontSize:12,marginBottom:6}},"仿真 (Gazebo)"),
            e("ul",{style:{listStyle:"disc",paddingLeft:16,fontSize:11,color:"#9ca3af",lineHeight:1.8}},
              e("li",null,"硬件接口: GazeboSystem"),
              e("li",null,"物理: Gazebo 引擎 (ODE)"),
              e("li",null,"跨域需要 ros_gz_bridge"),
              e("li",null,"世界/模型可随意修改")
            )
          ),
          e("div",{style:{marginBottom:14}},
            e("div",{style:{color:"var(--rd)",fontFamily:"JetBrains Mono",fontSize:12,marginBottom:6}},"真机 (无 Gazebo)"),
            e("ul",{style:{listStyle:"disc",paddingLeft:16,fontSize:11,color:"#9ca3af",lineHeight:1.8}},
              e("li",null,"硬件接口: 自定义实现"),
              e("li",null,"物理: 真实世界"),
              e("li",null,"全 DDS 通信，无需 bridge"),
              e("li",null,"需要真实电机/编码器/IMU")
            )
          ),
          e("div",{style:{paddingTop:12,borderTop:"1px solid var(--bdr)",marginBottom:14}},
            e("div",{style:{color:"var(--gr)",fontFamily:"JetBrains Mono",fontSize:12,marginBottom:6}},"两者完全相同"),
            e("ul",{style:{listStyle:"disc",paddingLeft:16,fontSize:11,color:"#9ca3af",lineHeight:1.8}},
              e("li",null,"diff_drive_controller 代码"),
              e("li",null,"controller_manager 配置"),
              e("li",null,"URDF 机器人描述"),
              e("li",null,"robot_state_publisher"),
              e("li",null,"/cmd_vel, /odom, /tf 话题")
            )
          ),
          e("div",{style:{paddingTop:12,borderTop:"1px solid var(--bdr)"}},
            e("div",{style:{color:"var(--pu)",fontFamily:"JetBrains Mono",fontSize:12,marginBottom:6}},"核心价值"),
            e("p",{style:{fontSize:11,color:"#9ca3af",lineHeight:1.6}},"ros2_control 的精髓：上层 controller 代码在仿真和真机间完全复用。切换时只换硬件接口实现，不改 controller 代码。")
          )
        )
      ),
      s.guideMode==="guided"?e(StepGuide):null,
      e(StatusBar)
    );
  }

  return e("div",{style:{height:"100%",display:"flex",flexDirection:"column"}},
    e(Header),e("div",{style:{flex:1,display:"flex",overflow:"hidden"}},
      e(Sidebar),e(Canvas),e(DetailPanel)
    ),s.guideMode==="guided"?e(StepGuide):null,e(StatusBar)
  );
}

// Mount
ReactDOM.createRoot(document.getElementById("root")).render(e(StoreProvider,null,e(App)));
'''

# Write final file
# Add w/h to component defs and fix key names (package->pkg, executable->exec)
comps_fixed = comps_js
# Replace 'package:' with 'pkg:' in the component defs
comps_fixed = re.sub(r'(\s+)package:', r'\1pkg:', comps_fixed)
# Replace 'executable:' with 'exec:' in the component defs
comps_fixed = re.sub(r'(\s+)executable:', r'\1exec:', comps_fixed)
# Add w/h after borderColor line
comps_fixed = re.sub(
    r"(borderColor: '[^']+'),",
    r"\1,\n    w: 170,\n    h: 65,",
    comps_fixed
)

with open(os.path.join(BASE, 'standalone.html'), 'w', encoding='utf-8') as f:
    f.write(head)
    f.write(comps_fixed)
    f.write('\n')
    f.write(steps_js)
    f.write('\n')
    f.write(logic)
    f.write(components_js)
    f.write('\n</script>\n</body>\n</html>')

print("standalone.html generated successfully!")
