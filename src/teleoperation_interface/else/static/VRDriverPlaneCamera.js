"use strict";(()=>{var xy=Object.create;var Kp=Object.defineProperty;var wy=Object.getOwnPropertyDescriptor;var Sy=Object.getOwnPropertyNames;var My=Object.getPrototypeOf,Ey=Object.prototype.hasOwnProperty;var Qi=(n,e)=>()=>(e||n((e={exports:{}}).exports,e),e.exports);var Cy=(n,e,t,i)=>{if(e&&typeof e=="object"||typeof e=="function")for(let r of Sy(e))!Ey.call(n,r)&&r!==t&&Kp(n,r,{get:()=>e[r],enumerable:!(i=wy(e,r))||i.enumerable});return n};var jp=(n,e,t)=>(t=n!=null?xy(My(n)):{},Cy(e||!n||!n.__esModule?Kp(t,"default",{value:n,enumerable:!0}):t,n));var lu=Qi((hb,em)=>{"use strict";var Qp=Object.getOwnPropertySymbols,Ty=Object.prototype.hasOwnProperty,by=Object.prototype.propertyIsEnumerable;function Ay(n){if(n==null)throw new TypeError("Object.assign cannot be called with null or undefined");return Object(n)}function Ry(){try{if(!Object.assign)return!1;var n=new String("abc");if(n[5]="de",Object.getOwnPropertyNames(n)[0]==="5")return!1;for(var e={},t=0;t<10;t++)e["_"+String.fromCharCode(t)]=t;var i=Object.getOwnPropertyNames(e).map(function(s){return e[s]});if(i.join("")!=="0123456789")return!1;var r={};return"abcdefghijklmnopqrst".split("").forEach(function(s){r[s]=s}),Object.keys(Object.assign({},r)).join("")==="abcdefghijklmnopqrst"}catch{return!1}}em.exports=Ry()?Object.assign:function(n,e){for(var t,i=Ay(n),r,s=1;s<arguments.length;s++){t=Object(arguments[s]);for(var o in t)Ty.call(t,o)&&(i[o]=t[o]);if(Qp){r=Qp(t);for(var a=0;a<r.length;a++)by.call(t,r[a])&&(i[r[a]]=t[r[a]])}}return i}});var gm=Qi(We=>{"use strict";var uu=lu(),Dr=60103,im=60106;We.Fragment=60107;We.StrictMode=60108;We.Profiler=60114;var rm=60109,sm=60110,om=60112;We.Suspense=60113;var am=60115,lm=60116;typeof Symbol=="function"&&Symbol.for&&(fn=Symbol.for,Dr=fn("react.element"),im=fn("react.portal"),We.Fragment=fn("react.fragment"),We.StrictMode=fn("react.strict_mode"),We.Profiler=fn("react.profiler"),rm=fn("react.provider"),sm=fn("react.context"),om=fn("react.forward_ref"),We.Suspense=fn("react.suspense"),am=fn("react.memo"),lm=fn("react.lazy"));var fn,tm=typeof Symbol=="function"&&Symbol.iterator;function Py(n){return n===null||typeof n!="object"?null:(n=tm&&n[tm]||n["@@iterator"],typeof n=="function"?n:null)}function $s(n){for(var e="https://reactjs.org/docs/error-decoder.html?invariant="+n,t=1;t<arguments.length;t++)e+="&args[]="+encodeURIComponent(arguments[t]);return"Minified React error #"+n+"; visit "+e+" for the full message or use the non-minified dev environment for full errors and additional helpful warnings."}var cm={isMounted:function(){return!1},enqueueForceUpdate:function(){},enqueueReplaceState:function(){},enqueueSetState:function(){}},um={};function Or(n,e,t){this.props=n,this.context=e,this.refs=um,this.updater=t||cm}Or.prototype.isReactComponent={};Or.prototype.setState=function(n,e){if(typeof n!="object"&&typeof n!="function"&&n!=null)throw Error($s(85));this.updater.enqueueSetState(this,n,e,"setState")};Or.prototype.forceUpdate=function(n){this.updater.enqueueForceUpdate(this,n,"forceUpdate")};function hm(){}hm.prototype=Or.prototype;function hu(n,e,t){this.props=n,this.context=e,this.refs=um,this.updater=t||cm}var du=hu.prototype=new hm;du.constructor=hu;uu(du,Or.prototype);du.isPureReactComponent=!0;var fu={current:null},dm=Object.prototype.hasOwnProperty,fm={key:!0,ref:!0,__self:!0,__source:!0};function pm(n,e,t){var i,r={},s=null,o=null;if(e!=null)for(i in e.ref!==void 0&&(o=e.ref),e.key!==void 0&&(s=""+e.key),e)dm.call(e,i)&&!fm.hasOwnProperty(i)&&(r[i]=e[i]);var a=arguments.length-2;if(a===1)r.children=t;else if(1<a){for(var l=Array(a),c=0;c<a;c++)l[c]=arguments[c+2];r.children=l}if(n&&n.defaultProps)for(i in a=n.defaultProps,a)r[i]===void 0&&(r[i]=a[i]);return{$$typeof:Dr,type:n,key:s,ref:o,props:r,_owner:fu.current}}function Iy(n,e){return{$$typeof:Dr,type:n.type,key:e,ref:n.ref,props:n.props,_owner:n._owner}}function pu(n){return typeof n=="object"&&n!==null&&n.$$typeof===Dr}function Ly(n){var e={"=":"=0",":":"=2"};return"$"+n.replace(/[=:]/g,function(t){return e[t]})}var nm=/\/+/g;function cu(n,e){return typeof n=="object"&&n!==null&&n.key!=null?Ly(""+n.key):e.toString(36)}function ya(n,e,t,i,r){var s=typeof n;(s==="undefined"||s==="boolean")&&(n=null);var o=!1;if(n===null)o=!0;else switch(s){case"string":case"number":o=!0;break;case"object":switch(n.$$typeof){case Dr:case im:o=!0}}if(o)return o=n,r=r(o),n=i===""?"."+cu(o,0):i,Array.isArray(r)?(t="",n!=null&&(t=n.replace(nm,"$&/")+"/"),ya(r,e,t,"",function(c){return c})):r!=null&&(pu(r)&&(r=Iy(r,t+(!r.key||o&&o.key===r.key?"":(""+r.key).replace(nm,"$&/")+"/")+n)),e.push(r)),1;if(o=0,i=i===""?".":i+":",Array.isArray(n))for(var a=0;a<n.length;a++){s=n[a];var l=i+cu(s,a);o+=ya(s,e,t,l,r)}else if(l=Py(n),typeof l=="function")for(n=l.call(n),a=0;!(s=n.next()).done;)s=s.value,l=i+cu(s,a++),o+=ya(s,e,t,l,r);else if(s==="object")throw e=""+n,Error($s(31,e==="[object Object]"?"object with keys {"+Object.keys(n).join(", ")+"}":e));return o}function va(n,e,t){if(n==null)return n;var i=[],r=0;return ya(n,i,"","",function(s){return e.call(t,s,r++)}),i}function Uy(n){if(n._status===-1){var e=n._result;e=e(),n._status=0,n._result=e,e.then(function(t){n._status===0&&(t=t.default,n._status=1,n._result=t)},function(t){n._status===0&&(n._status=2,n._result=t)})}if(n._status===1)return n._result;throw n._result}var mm={current:null};function $n(){var n=mm.current;if(n===null)throw Error($s(321));return n}var Ny={ReactCurrentDispatcher:mm,ReactCurrentBatchConfig:{transition:0},ReactCurrentOwner:fu,IsSomeRendererActing:{current:!1},assign:uu};We.Children={map:va,forEach:function(n,e,t){va(n,function(){e.apply(this,arguments)},t)},count:function(n){var e=0;return va(n,function(){e++}),e},toArray:function(n){return va(n,function(e){return e})||[]},only:function(n){if(!pu(n))throw Error($s(143));return n}};We.Component=Or;We.PureComponent=hu;We.__SECRET_INTERNALS_DO_NOT_USE_OR_YOU_WILL_BE_FIRED=Ny;We.cloneElement=function(n,e,t){if(n==null)throw Error($s(267,n));var i=uu({},n.props),r=n.key,s=n.ref,o=n._owner;if(e!=null){if(e.ref!==void 0&&(s=e.ref,o=fu.current),e.key!==void 0&&(r=""+e.key),n.type&&n.type.defaultProps)var a=n.type.defaultProps;for(l in e)dm.call(e,l)&&!fm.hasOwnProperty(l)&&(i[l]=e[l]===void 0&&a!==void 0?a[l]:e[l])}var l=arguments.length-2;if(l===1)i.children=t;else if(1<l){a=Array(l);for(var c=0;c<l;c++)a[c]=arguments[c+2];i.children=a}return{$$typeof:Dr,type:n.type,key:r,ref:s,props:i,_owner:o}};We.createContext=function(n,e){return e===void 0&&(e=null),n={$$typeof:sm,_calculateChangedBits:e,_currentValue:n,_currentValue2:n,_threadCount:0,Provider:null,Consumer:null},n.Provider={$$typeof:rm,_context:n},n.Consumer=n};We.createElement=pm;We.createFactory=function(n){var e=pm.bind(null,n);return e.type=n,e};We.createRef=function(){return{current:null}};We.forwardRef=function(n){return{$$typeof:om,render:n}};We.isValidElement=pu;We.lazy=function(n){return{$$typeof:lm,_payload:{_status:-1,_result:n},_init:Uy}};We.memo=function(n,e){return{$$typeof:am,type:n,compare:e===void 0?null:e}};We.useCallback=function(n,e){return $n().useCallback(n,e)};We.useContext=function(n,e){return $n().useContext(n,e)};We.useDebugValue=function(){};We.useEffect=function(n,e){return $n().useEffect(n,e)};We.useImperativeHandle=function(n,e,t){return $n().useImperativeHandle(n,e,t)};We.useLayoutEffect=function(n,e){return $n().useLayoutEffect(n,e)};We.useMemo=function(n,e){return $n().useMemo(n,e)};We.useReducer=function(n,e,t){return $n().useReducer(n,e,t)};We.useRef=function(n){return $n().useRef(n)};We.useState=function(n){return $n().useState(n)};We.version="17.0.2"});var mu=Qi((fb,_m)=>{"use strict";_m.exports=gm()});var Mm=Qi(Ze=>{"use strict";var Br,Ks,Ma,Su;typeof performance=="object"&&typeof performance.now=="function"?(vm=performance,Ze.unstable_now=function(){return vm.now()}):(gu=Date,ym=gu.now(),Ze.unstable_now=function(){return gu.now()-ym});var vm,gu,ym;typeof window>"u"||typeof MessageChannel!="function"?(Fr=null,_u=null,vu=function(){if(Fr!==null)try{var n=Ze.unstable_now();Fr(!0,n),Fr=null}catch(e){throw setTimeout(vu,0),e}},Br=function(n){Fr!==null?setTimeout(Br,0,n):(Fr=n,setTimeout(vu,0))},Ks=function(n,e){_u=setTimeout(n,e)},Ma=function(){clearTimeout(_u)},Ze.unstable_shouldYield=function(){return!1},Su=Ze.unstable_forceFrameRate=function(){}):(xm=window.setTimeout,wm=window.clearTimeout,typeof console<"u"&&(Sm=window.cancelAnimationFrame,typeof window.requestAnimationFrame!="function"&&console.error("This browser doesn't support requestAnimationFrame. Make sure that you load a polyfill in older browsers. https://reactjs.org/link/react-polyfills"),typeof Sm!="function"&&console.error("This browser doesn't support cancelAnimationFrame. Make sure that you load a polyfill in older browsers. https://reactjs.org/link/react-polyfills")),Zs=!1,Js=null,xa=-1,yu=5,xu=0,Ze.unstable_shouldYield=function(){return Ze.unstable_now()>=xu},Su=function(){},Ze.unstable_forceFrameRate=function(n){0>n||125<n?console.error("forceFrameRate takes a positive int between 0 and 125, forcing frame rates higher than 125 fps is not supported"):yu=0<n?Math.floor(1e3/n):5},wu=new MessageChannel,wa=wu.port2,wu.port1.onmessage=function(){if(Js!==null){var n=Ze.unstable_now();xu=n+yu;try{Js(!0,n)?wa.postMessage(null):(Zs=!1,Js=null)}catch(e){throw wa.postMessage(null),e}}else Zs=!1},Br=function(n){Js=n,Zs||(Zs=!0,wa.postMessage(null))},Ks=function(n,e){xa=xm(function(){n(Ze.unstable_now())},e)},Ma=function(){wm(xa),xa=-1});var Fr,_u,vu,xm,wm,Sm,Zs,Js,xa,yu,xu,wu,wa;function Mu(n,e){var t=n.length;n.push(e);e:for(;;){var i=t-1>>>1,r=n[i];if(r!==void 0&&0<Sa(r,e))n[i]=e,n[t]=r,t=i;else break e}}function Tn(n){return n=n[0],n===void 0?null:n}function Ea(n){var e=n[0];if(e!==void 0){var t=n.pop();if(t!==e){n[0]=t;e:for(var i=0,r=n.length;i<r;){var s=2*(i+1)-1,o=n[s],a=s+1,l=n[a];if(o!==void 0&&0>Sa(o,t))l!==void 0&&0>Sa(l,o)?(n[i]=l,n[a]=t,i=a):(n[i]=o,n[s]=t,i=s);else if(l!==void 0&&0>Sa(l,t))n[i]=l,n[a]=t,i=a;else break e}}return e}return null}function Sa(n,e){var t=n.sortIndex-e.sortIndex;return t!==0?t:n.id-e.id}var Fn=[],_i=[],Dy=1,pn=null,Nt=3,Ca=!1,er=!1,js=!1;function Eu(n){for(var e=Tn(_i);e!==null;){if(e.callback===null)Ea(_i);else if(e.startTime<=n)Ea(_i),e.sortIndex=e.expirationTime,Mu(Fn,e);else break;e=Tn(_i)}}function Cu(n){if(js=!1,Eu(n),!er)if(Tn(Fn)!==null)er=!0,Br(Tu);else{var e=Tn(_i);e!==null&&Ks(Cu,e.startTime-n)}}function Tu(n,e){er=!1,js&&(js=!1,Ma()),Ca=!0;var t=Nt;try{for(Eu(e),pn=Tn(Fn);pn!==null&&(!(pn.expirationTime>e)||n&&!Ze.unstable_shouldYield());){var i=pn.callback;if(typeof i=="function"){pn.callback=null,Nt=pn.priorityLevel;var r=i(pn.expirationTime<=e);e=Ze.unstable_now(),typeof r=="function"?pn.callback=r:pn===Tn(Fn)&&Ea(Fn),Eu(e)}else Ea(Fn);pn=Tn(Fn)}if(pn!==null)var s=!0;else{var o=Tn(_i);o!==null&&Ks(Cu,o.startTime-e),s=!1}return s}finally{pn=null,Nt=t,Ca=!1}}var Oy=Su;Ze.unstable_IdlePriority=5;Ze.unstable_ImmediatePriority=1;Ze.unstable_LowPriority=4;Ze.unstable_NormalPriority=3;Ze.unstable_Profiling=null;Ze.unstable_UserBlockingPriority=2;Ze.unstable_cancelCallback=function(n){n.callback=null};Ze.unstable_continueExecution=function(){er||Ca||(er=!0,Br(Tu))};Ze.unstable_getCurrentPriorityLevel=function(){return Nt};Ze.unstable_getFirstCallbackNode=function(){return Tn(Fn)};Ze.unstable_next=function(n){switch(Nt){case 1:case 2:case 3:var e=3;break;default:e=Nt}var t=Nt;Nt=e;try{return n()}finally{Nt=t}};Ze.unstable_pauseExecution=function(){};Ze.unstable_requestPaint=Oy;Ze.unstable_runWithPriority=function(n,e){switch(n){case 1:case 2:case 3:case 4:case 5:break;default:n=3}var t=Nt;Nt=n;try{return e()}finally{Nt=t}};Ze.unstable_scheduleCallback=function(n,e,t){var i=Ze.unstable_now();switch(typeof t=="object"&&t!==null?(t=t.delay,t=typeof t=="number"&&0<t?i+t:i):t=i,n){case 1:var r=-1;break;case 2:r=250;break;case 5:r=1073741823;break;case 4:r=1e4;break;default:r=5e3}return r=t+r,n={id:Dy++,callback:e,priorityLevel:n,startTime:t,expirationTime:r,sortIndex:-1},t>i?(n.sortIndex=t,Mu(_i,n),Tn(Fn)===null&&n===Tn(_i)&&(js?Ma():js=!0,Ks(Cu,t-i))):(n.sortIndex=r,Mu(Fn,n),er||Ca||(er=!0,Br(Tu))),n};Ze.unstable_wrapCallback=function(n){var e=Nt;return function(){var t=Nt;Nt=e;try{return n.apply(this,arguments)}finally{Nt=t}}}});var Cm=Qi((mb,Em)=>{"use strict";Em.exports=Mm()});var h_=Qi(xn=>{"use strict";var vl=mu(),st=lu(),Ct=Cm();function j(n){for(var e="https://reactjs.org/docs/error-decoder.html?invariant="+n,t=1;t<arguments.length;t++)e+="&args[]="+encodeURIComponent(arguments[t]);return"Minified React error #"+n+"; visit "+e+" for the full message or use the non-minified dev environment for full errors and additional helpful warnings."}if(!vl)throw Error(j(227));var Bg=new Set,Ro={};function hr(n,e){ss(n,e),ss(n+"Capture",e)}function ss(n,e){for(Ro[n]=e,n=0;n<e.length;n++)Bg.add(e[n])}var ei=!(typeof window>"u"||typeof window.document>"u"||typeof window.document.createElement>"u"),Fy=/^[:A-Z_a-z\u00C0-\u00D6\u00D8-\u00F6\u00F8-\u02FF\u0370-\u037D\u037F-\u1FFF\u200C-\u200D\u2070-\u218F\u2C00-\u2FEF\u3001-\uD7FF\uF900-\uFDCF\uFDF0-\uFFFD][:A-Z_a-z\u00C0-\u00D6\u00D8-\u00F6\u00F8-\u02FF\u0370-\u037D\u037F-\u1FFF\u200C-\u200D\u2070-\u218F\u2C00-\u2FEF\u3001-\uD7FF\uF900-\uFDCF\uFDF0-\uFFFD\-.0-9\u00B7\u0300-\u036F\u203F-\u2040]*$/,Tm=Object.prototype.hasOwnProperty,bm={},Am={};function By(n){return Tm.call(Am,n)?!0:Tm.call(bm,n)?!1:Fy.test(n)?Am[n]=!0:(bm[n]=!0,!1)}function ky(n,e,t,i){if(t!==null&&t.type===0)return!1;switch(typeof e){case"function":case"symbol":return!0;case"boolean":return i?!1:t!==null?!t.acceptsBooleans:(n=n.toLowerCase().slice(0,5),n!=="data-"&&n!=="aria-");default:return!1}}function zy(n,e,t,i){if(e===null||typeof e>"u"||ky(n,e,t,i))return!0;if(i)return!1;if(t!==null)switch(t.type){case 3:return!e;case 4:return e===!1;case 5:return isNaN(e);case 6:return isNaN(e)||1>e}return!1}function Wt(n,e,t,i,r,s,o){this.acceptsBooleans=e===2||e===3||e===4,this.attributeName=i,this.attributeNamespace=r,this.mustUseProperty=t,this.propertyName=n,this.type=e,this.sanitizeURL=s,this.removeEmptyString=o}var It={};"children dangerouslySetInnerHTML defaultValue defaultChecked innerHTML suppressContentEditableWarning suppressHydrationWarning style".split(" ").forEach(function(n){It[n]=new Wt(n,0,!1,n,null,!1,!1)});[["acceptCharset","accept-charset"],["className","class"],["htmlFor","for"],["httpEquiv","http-equiv"]].forEach(function(n){var e=n[0];It[e]=new Wt(e,1,!1,n[1],null,!1,!1)});["contentEditable","draggable","spellCheck","value"].forEach(function(n){It[n]=new Wt(n,2,!1,n.toLowerCase(),null,!1,!1)});["autoReverse","externalResourcesRequired","focusable","preserveAlpha"].forEach(function(n){It[n]=new Wt(n,2,!1,n,null,!1,!1)});"allowFullScreen async autoFocus autoPlay controls default defer disabled disablePictureInPicture disableRemotePlayback formNoValidate hidden loop noModule noValidate open playsInline readOnly required reversed scoped seamless itemScope".split(" ").forEach(function(n){It[n]=new Wt(n,3,!1,n.toLowerCase(),null,!1,!1)});["checked","multiple","muted","selected"].forEach(function(n){It[n]=new Wt(n,3,!0,n,null,!1,!1)});["capture","download"].forEach(function(n){It[n]=new Wt(n,4,!1,n,null,!1,!1)});["cols","rows","size","span"].forEach(function(n){It[n]=new Wt(n,6,!1,n,null,!1,!1)});["rowSpan","start"].forEach(function(n){It[n]=new Wt(n,5,!1,n.toLowerCase(),null,!1,!1)});var Dh=/[\-:]([a-z])/g;function Oh(n){return n[1].toUpperCase()}"accent-height alignment-baseline arabic-form baseline-shift cap-height clip-path clip-rule color-interpolation color-interpolation-filters color-profile color-rendering dominant-baseline enable-background fill-opacity fill-rule flood-color flood-opacity font-family font-size font-size-adjust font-stretch font-style font-variant font-weight glyph-name glyph-orientation-horizontal glyph-orientation-vertical horiz-adv-x horiz-origin-x image-rendering letter-spacing lighting-color marker-end marker-mid marker-start overline-position overline-thickness paint-order panose-1 pointer-events rendering-intent shape-rendering stop-color stop-opacity strikethrough-position strikethrough-thickness stroke-dasharray stroke-dashoffset stroke-linecap stroke-linejoin stroke-miterlimit stroke-opacity stroke-width text-anchor text-decoration text-rendering underline-position underline-thickness unicode-bidi unicode-range units-per-em v-alphabetic v-hanging v-ideographic v-mathematical vector-effect vert-adv-y vert-origin-x vert-origin-y word-spacing writing-mode xmlns:xlink x-height".split(" ").forEach(function(n){var e=n.replace(Dh,Oh);It[e]=new Wt(e,1,!1,n,null,!1,!1)});"xlink:actuate xlink:arcrole xlink:role xlink:show xlink:title xlink:type".split(" ").forEach(function(n){var e=n.replace(Dh,Oh);It[e]=new Wt(e,1,!1,n,"http://www.w3.org/1999/xlink",!1,!1)});["xml:base","xml:lang","xml:space"].forEach(function(n){var e=n.replace(Dh,Oh);It[e]=new Wt(e,1,!1,n,"http://www.w3.org/XML/1998/namespace",!1,!1)});["tabIndex","crossOrigin"].forEach(function(n){It[n]=new Wt(n,1,!1,n.toLowerCase(),null,!1,!1)});It.xlinkHref=new Wt("xlinkHref",1,!1,"xlink:href","http://www.w3.org/1999/xlink",!0,!1);["src","href","action","formAction"].forEach(function(n){It[n]=new Wt(n,1,!1,n.toLowerCase(),null,!0,!0)});function Fh(n,e,t,i){var r=It.hasOwnProperty(e)?It[e]:null,s=r!==null?r.type===0:i?!1:!(!(2<e.length)||e[0]!=="o"&&e[0]!=="O"||e[1]!=="n"&&e[1]!=="N");s||(zy(e,t,r,i)&&(t=null),i||r===null?By(e)&&(t===null?n.removeAttribute(e):n.setAttribute(e,""+t)):r.mustUseProperty?n[r.propertyName]=t===null?r.type===3?!1:"":t:(e=r.attributeName,i=r.attributeNamespace,t===null?n.removeAttribute(e):(r=r.type,t=r===3||r===4&&t===!0?"":""+t,i?n.setAttributeNS(i,e,t):n.setAttribute(e,t))))}var dr=vl.__SECRET_INTERNALS_DO_NOT_USE_OR_YOU_WILL_BE_FIRED,ho=60103,nr=60106,yi=60107,Bh=60108,_o=60114,kh=60109,zh=60110,yl=60112,vo=60113,Za=60120,xl=60115,Vh=60116,Hh=60121,Gh=60128,kg=60129,Wh=60130,Zu=60131;typeof Symbol=="function"&&Symbol.for&&(yt=Symbol.for,ho=yt("react.element"),nr=yt("react.portal"),yi=yt("react.fragment"),Bh=yt("react.strict_mode"),_o=yt("react.profiler"),kh=yt("react.provider"),zh=yt("react.context"),yl=yt("react.forward_ref"),vo=yt("react.suspense"),Za=yt("react.suspense_list"),xl=yt("react.memo"),Vh=yt("react.lazy"),Hh=yt("react.block"),yt("react.scope"),Gh=yt("react.opaque.id"),kg=yt("react.debug_trace_mode"),Wh=yt("react.offscreen"),Zu=yt("react.legacy_hidden"));var yt,Rm=typeof Symbol=="function"&&Symbol.iterator;function Qs(n){return n===null||typeof n!="object"?null:(n=Rm&&n[Rm]||n["@@iterator"],typeof n=="function"?n:null)}var bu;function fo(n){if(bu===void 0)try{throw Error()}catch(t){var e=t.stack.trim().match(/\n( *(at )?)/);bu=e&&e[1]||""}return`
`+bu+n}var Au=!1;function Ta(n,e){if(!n||Au)return"";Au=!0;var t=Error.prepareStackTrace;Error.prepareStackTrace=void 0;try{if(e)if(e=function(){throw Error()},Object.defineProperty(e.prototype,"props",{set:function(){throw Error()}}),typeof Reflect=="object"&&Reflect.construct){try{Reflect.construct(e,[])}catch(l){var i=l}Reflect.construct(n,[],e)}else{try{e.call()}catch(l){i=l}n.call(e.prototype)}else{try{throw Error()}catch(l){i=l}n()}}catch(l){if(l&&i&&typeof l.stack=="string"){for(var r=l.stack.split(`
`),s=i.stack.split(`
`),o=r.length-1,a=s.length-1;1<=o&&0<=a&&r[o]!==s[a];)a--;for(;1<=o&&0<=a;o--,a--)if(r[o]!==s[a]){if(o!==1||a!==1)do if(o--,a--,0>a||r[o]!==s[a])return`
`+r[o].replace(" at new "," at ");while(1<=o&&0<=a);break}}}finally{Au=!1,Error.prepareStackTrace=t}return(n=n?n.displayName||n.name:"")?fo(n):""}function Vy(n){switch(n.tag){case 5:return fo(n.type);case 16:return fo("Lazy");case 13:return fo("Suspense");case 19:return fo("SuspenseList");case 0:case 2:case 15:return n=Ta(n.type,!1),n;case 11:return n=Ta(n.type.render,!1),n;case 22:return n=Ta(n.type._render,!1),n;case 1:return n=Ta(n.type,!0),n;default:return""}}function Zr(n){if(n==null)return null;if(typeof n=="function")return n.displayName||n.name||null;if(typeof n=="string")return n;switch(n){case yi:return"Fragment";case nr:return"Portal";case _o:return"Profiler";case Bh:return"StrictMode";case vo:return"Suspense";case Za:return"SuspenseList"}if(typeof n=="object")switch(n.$$typeof){case zh:return(n.displayName||"Context")+".Consumer";case kh:return(n._context.displayName||"Context")+".Provider";case yl:var e=n.render;return e=e.displayName||e.name||"",n.displayName||(e!==""?"ForwardRef("+e+")":"ForwardRef");case xl:return Zr(n.type);case Hh:return Zr(n._render);case Vh:e=n._payload,n=n._init;try{return Zr(n(e))}catch{}}return null}function Ii(n){switch(typeof n){case"boolean":case"number":case"object":case"string":case"undefined":return n;default:return""}}function zg(n){var e=n.type;return(n=n.nodeName)&&n.toLowerCase()==="input"&&(e==="checkbox"||e==="radio")}function Hy(n){var e=zg(n)?"checked":"value",t=Object.getOwnPropertyDescriptor(n.constructor.prototype,e),i=""+n[e];if(!n.hasOwnProperty(e)&&typeof t<"u"&&typeof t.get=="function"&&typeof t.set=="function"){var r=t.get,s=t.set;return Object.defineProperty(n,e,{configurable:!0,get:function(){return r.call(this)},set:function(o){i=""+o,s.call(this,o)}}),Object.defineProperty(n,e,{enumerable:t.enumerable}),{getValue:function(){return i},setValue:function(o){i=""+o},stopTracking:function(){n._valueTracker=null,delete n[e]}}}}function ba(n){n._valueTracker||(n._valueTracker=Hy(n))}function Vg(n){if(!n)return!1;var e=n._valueTracker;if(!e)return!0;var t=e.getValue(),i="";return n&&(i=zg(n)?n.checked?"true":"false":n.value),n=i,n!==t?(e.setValue(n),!0):!1}function Ja(n){if(n=n||(typeof document<"u"?document:void 0),typeof n>"u")return null;try{return n.activeElement||n.body}catch{return n.body}}function Ju(n,e){var t=e.checked;return st({},e,{defaultChecked:void 0,defaultValue:void 0,value:void 0,checked:t??n._wrapperState.initialChecked})}function Pm(n,e){var t=e.defaultValue==null?"":e.defaultValue,i=e.checked!=null?e.checked:e.defaultChecked;t=Ii(e.value!=null?e.value:t),n._wrapperState={initialChecked:i,initialValue:t,controlled:e.type==="checkbox"||e.type==="radio"?e.checked!=null:e.value!=null}}function Hg(n,e){e=e.checked,e!=null&&Fh(n,"checked",e,!1)}function Ku(n,e){Hg(n,e);var t=Ii(e.value),i=e.type;if(t!=null)i==="number"?(t===0&&n.value===""||n.value!=t)&&(n.value=""+t):n.value!==""+t&&(n.value=""+t);else if(i==="submit"||i==="reset"){n.removeAttribute("value");return}e.hasOwnProperty("value")?ju(n,e.type,t):e.hasOwnProperty("defaultValue")&&ju(n,e.type,Ii(e.defaultValue)),e.checked==null&&e.defaultChecked!=null&&(n.defaultChecked=!!e.defaultChecked)}function Im(n,e,t){if(e.hasOwnProperty("value")||e.hasOwnProperty("defaultValue")){var i=e.type;if(!(i!=="submit"&&i!=="reset"||e.value!==void 0&&e.value!==null))return;e=""+n._wrapperState.initialValue,t||e===n.value||(n.value=e),n.defaultValue=e}t=n.name,t!==""&&(n.name=""),n.defaultChecked=!!n._wrapperState.initialChecked,t!==""&&(n.name=t)}function ju(n,e,t){(e!=="number"||Ja(n.ownerDocument)!==n)&&(t==null?n.defaultValue=""+n._wrapperState.initialValue:n.defaultValue!==""+t&&(n.defaultValue=""+t))}function Gy(n){var e="";return vl.Children.forEach(n,function(t){t!=null&&(e+=t)}),e}function Qu(n,e){return n=st({children:void 0},e),(e=Gy(e.children))&&(n.children=e),n}function Jr(n,e,t,i){if(n=n.options,e){e={};for(var r=0;r<t.length;r++)e["$"+t[r]]=!0;for(t=0;t<n.length;t++)r=e.hasOwnProperty("$"+n[t].value),n[t].selected!==r&&(n[t].selected=r),r&&i&&(n[t].defaultSelected=!0)}else{for(t=""+Ii(t),e=null,r=0;r<n.length;r++){if(n[r].value===t){n[r].selected=!0,i&&(n[r].defaultSelected=!0);return}e!==null||n[r].disabled||(e=n[r])}e!==null&&(e.selected=!0)}}function eh(n,e){if(e.dangerouslySetInnerHTML!=null)throw Error(j(91));return st({},e,{value:void 0,defaultValue:void 0,children:""+n._wrapperState.initialValue})}function Lm(n,e){var t=e.value;if(t==null){if(t=e.children,e=e.defaultValue,t!=null){if(e!=null)throw Error(j(92));if(Array.isArray(t)){if(!(1>=t.length))throw Error(j(93));t=t[0]}e=t}e==null&&(e=""),t=e}n._wrapperState={initialValue:Ii(t)}}function Gg(n,e){var t=Ii(e.value),i=Ii(e.defaultValue);t!=null&&(t=""+t,t!==n.value&&(n.value=t),e.defaultValue==null&&n.defaultValue!==t&&(n.defaultValue=t)),i!=null&&(n.defaultValue=""+i)}function Um(n){var e=n.textContent;e===n._wrapperState.initialValue&&e!==""&&e!==null&&(n.value=e)}var th={html:"http://www.w3.org/1999/xhtml",mathml:"http://www.w3.org/1998/Math/MathML",svg:"http://www.w3.org/2000/svg"};function Wg(n){switch(n){case"svg":return"http://www.w3.org/2000/svg";case"math":return"http://www.w3.org/1998/Math/MathML";default:return"http://www.w3.org/1999/xhtml"}}function nh(n,e){return n==null||n==="http://www.w3.org/1999/xhtml"?Wg(e):n==="http://www.w3.org/2000/svg"&&e==="foreignObject"?"http://www.w3.org/1999/xhtml":n}var Aa,Xg=function(n){return typeof MSApp<"u"&&MSApp.execUnsafeLocalFunction?function(e,t,i,r){MSApp.execUnsafeLocalFunction(function(){return n(e,t,i,r)})}:n}(function(n,e){if(n.namespaceURI!==th.svg||"innerHTML"in n)n.innerHTML=e;else{for(Aa=Aa||document.createElement("div"),Aa.innerHTML="<svg>"+e.valueOf().toString()+"</svg>",e=Aa.firstChild;n.firstChild;)n.removeChild(n.firstChild);for(;e.firstChild;)n.appendChild(e.firstChild)}});function Po(n,e){if(e){var t=n.firstChild;if(t&&t===n.lastChild&&t.nodeType===3){t.nodeValue=e;return}}n.textContent=e}var yo={animationIterationCount:!0,borderImageOutset:!0,borderImageSlice:!0,borderImageWidth:!0,boxFlex:!0,boxFlexGroup:!0,boxOrdinalGroup:!0,columnCount:!0,columns:!0,flex:!0,flexGrow:!0,flexPositive:!0,flexShrink:!0,flexNegative:!0,flexOrder:!0,gridArea:!0,gridRow:!0,gridRowEnd:!0,gridRowSpan:!0,gridRowStart:!0,gridColumn:!0,gridColumnEnd:!0,gridColumnSpan:!0,gridColumnStart:!0,fontWeight:!0,lineClamp:!0,lineHeight:!0,opacity:!0,order:!0,orphans:!0,tabSize:!0,widows:!0,zIndex:!0,zoom:!0,fillOpacity:!0,floodOpacity:!0,stopOpacity:!0,strokeDasharray:!0,strokeDashoffset:!0,strokeMiterlimit:!0,strokeOpacity:!0,strokeWidth:!0},Wy=["Webkit","ms","Moz","O"];Object.keys(yo).forEach(function(n){Wy.forEach(function(e){e=e+n.charAt(0).toUpperCase()+n.substring(1),yo[e]=yo[n]})});function Yg(n,e,t){return e==null||typeof e=="boolean"||e===""?"":t||typeof e!="number"||e===0||yo.hasOwnProperty(n)&&yo[n]?(""+e).trim():e+"px"}function qg(n,e){n=n.style;for(var t in e)if(e.hasOwnProperty(t)){var i=t.indexOf("--")===0,r=Yg(t,e[t],i);t==="float"&&(t="cssFloat"),i?n.setProperty(t,r):n[t]=r}}var Xy=st({menuitem:!0},{area:!0,base:!0,br:!0,col:!0,embed:!0,hr:!0,img:!0,input:!0,keygen:!0,link:!0,meta:!0,param:!0,source:!0,track:!0,wbr:!0});function ih(n,e){if(e){if(Xy[n]&&(e.children!=null||e.dangerouslySetInnerHTML!=null))throw Error(j(137,n));if(e.dangerouslySetInnerHTML!=null){if(e.children!=null)throw Error(j(60));if(!(typeof e.dangerouslySetInnerHTML=="object"&&"__html"in e.dangerouslySetInnerHTML))throw Error(j(61))}if(e.style!=null&&typeof e.style!="object")throw Error(j(62))}}function rh(n,e){if(n.indexOf("-")===-1)return typeof e.is=="string";switch(n){case"annotation-xml":case"color-profile":case"font-face":case"font-face-src":case"font-face-uri":case"font-face-format":case"font-face-name":case"missing-glyph":return!1;default:return!0}}function Xh(n){return n=n.target||n.srcElement||window,n.correspondingUseElement&&(n=n.correspondingUseElement),n.nodeType===3?n.parentNode:n}var sh=null,Kr=null,jr=null;function Nm(n){if(n=Wo(n)){if(typeof sh!="function")throw Error(j(280));var e=n.stateNode;e&&(e=Tl(e),sh(n.stateNode,n.type,e))}}function $g(n){Kr?jr?jr.push(n):jr=[n]:Kr=n}function Zg(){if(Kr){var n=Kr,e=jr;if(jr=Kr=null,Nm(n),e)for(n=0;n<e.length;n++)Nm(e[n])}}function Yh(n,e){return n(e)}function Jg(n,e,t,i,r){return n(e,t,i,r)}function qh(){}var Kg=Yh,ir=!1,Ru=!1;function $h(){(Kr!==null||jr!==null)&&(qh(),Zg())}function Yy(n,e,t){if(Ru)return n(e,t);Ru=!0;try{return Kg(n,e,t)}finally{Ru=!1,$h()}}function Io(n,e){var t=n.stateNode;if(t===null)return null;var i=Tl(t);if(i===null)return null;t=i[e];e:switch(e){case"onClick":case"onClickCapture":case"onDoubleClick":case"onDoubleClickCapture":case"onMouseDown":case"onMouseDownCapture":case"onMouseMove":case"onMouseMoveCapture":case"onMouseUp":case"onMouseUpCapture":case"onMouseEnter":(i=!i.disabled)||(n=n.type,i=!(n==="button"||n==="input"||n==="select"||n==="textarea")),n=!i;break e;default:n=!1}if(n)return null;if(t&&typeof t!="function")throw Error(j(231,e,typeof t));return t}var oh=!1;if(ei)try{kr={},Object.defineProperty(kr,"passive",{get:function(){oh=!0}}),window.addEventListener("test",kr,kr),window.removeEventListener("test",kr,kr)}catch{oh=!1}var kr;function qy(n,e,t,i,r,s,o,a,l){var c=Array.prototype.slice.call(arguments,3);try{e.apply(t,c)}catch(u){this.onError(u)}}var xo=!1,Ka=null,ja=!1,ah=null,$y={onError:function(n){xo=!0,Ka=n}};function Zy(n,e,t,i,r,s,o,a,l){xo=!1,Ka=null,qy.apply($y,arguments)}function Jy(n,e,t,i,r,s,o,a,l){if(Zy.apply(this,arguments),xo){if(xo){var c=Ka;xo=!1,Ka=null}else throw Error(j(198));ja||(ja=!0,ah=c)}}function fr(n){var e=n,t=n;if(n.alternate)for(;e.return;)e=e.return;else{n=e;do e=n,e.flags&1026&&(t=e.return),n=e.return;while(n)}return e.tag===3?t:null}function jg(n){if(n.tag===13){var e=n.memoizedState;if(e===null&&(n=n.alternate,n!==null&&(e=n.memoizedState)),e!==null)return e.dehydrated}return null}function Dm(n){if(fr(n)!==n)throw Error(j(188))}function Ky(n){var e=n.alternate;if(!e){if(e=fr(n),e===null)throw Error(j(188));return e!==n?null:n}for(var t=n,i=e;;){var r=t.return;if(r===null)break;var s=r.alternate;if(s===null){if(i=r.return,i!==null){t=i;continue}break}if(r.child===s.child){for(s=r.child;s;){if(s===t)return Dm(r),n;if(s===i)return Dm(r),e;s=s.sibling}throw Error(j(188))}if(t.return!==i.return)t=r,i=s;else{for(var o=!1,a=r.child;a;){if(a===t){o=!0,t=r,i=s;break}if(a===i){o=!0,i=r,t=s;break}a=a.sibling}if(!o){for(a=s.child;a;){if(a===t){o=!0,t=s,i=r;break}if(a===i){o=!0,i=s,t=r;break}a=a.sibling}if(!o)throw Error(j(189))}}if(t.alternate!==i)throw Error(j(190))}if(t.tag!==3)throw Error(j(188));return t.stateNode.current===t?n:e}function Qg(n){if(n=Ky(n),!n)return null;for(var e=n;;){if(e.tag===5||e.tag===6)return e;if(e.child)e.child.return=e,e=e.child;else{if(e===n)break;for(;!e.sibling;){if(!e.return||e.return===n)return null;e=e.return}e.sibling.return=e.return,e=e.sibling}}return null}function Om(n,e){for(var t=n.alternate;e!==null;){if(e===n||e===t)return!0;e=e.return}return!1}var e0,Zh,t0,n0,lh=!1,Bn=[],Mi=null,Ei=null,Ci=null,Lo=new Map,Uo=new Map,eo=[],Fm="mousedown mouseup touchcancel touchend touchstart auxclick dblclick pointercancel pointerdown pointerup dragend dragstart drop compositionend compositionstart keydown keypress keyup input textInput copy cut paste click change contextmenu reset submit".split(" ");function ch(n,e,t,i,r){return{blockedOn:n,domEventName:e,eventSystemFlags:t|16,nativeEvent:r,targetContainers:[i]}}function Bm(n,e){switch(n){case"focusin":case"focusout":Mi=null;break;case"dragenter":case"dragleave":Ei=null;break;case"mouseover":case"mouseout":Ci=null;break;case"pointerover":case"pointerout":Lo.delete(e.pointerId);break;case"gotpointercapture":case"lostpointercapture":Uo.delete(e.pointerId)}}function to(n,e,t,i,r,s){return n===null||n.nativeEvent!==s?(n=ch(e,t,i,r,s),e!==null&&(e=Wo(e),e!==null&&Zh(e)),n):(n.eventSystemFlags|=i,e=n.targetContainers,r!==null&&e.indexOf(r)===-1&&e.push(r),n)}function jy(n,e,t,i,r){switch(e){case"focusin":return Mi=to(Mi,n,e,t,i,r),!0;case"dragenter":return Ei=to(Ei,n,e,t,i,r),!0;case"mouseover":return Ci=to(Ci,n,e,t,i,r),!0;case"pointerover":var s=r.pointerId;return Lo.set(s,to(Lo.get(s)||null,n,e,t,i,r)),!0;case"gotpointercapture":return s=r.pointerId,Uo.set(s,to(Uo.get(s)||null,n,e,t,i,r)),!0}return!1}function Qy(n){var e=rr(n.target);if(e!==null){var t=fr(e);if(t!==null){if(e=t.tag,e===13){if(e=jg(t),e!==null){n.blockedOn=e,n0(n.lanePriority,function(){Ct.unstable_runWithPriority(n.priority,function(){t0(t)})});return}}else if(e===3&&t.stateNode.hydrate){n.blockedOn=t.tag===3?t.stateNode.containerInfo:null;return}}}n.blockedOn=null}function ka(n){if(n.blockedOn!==null)return!1;for(var e=n.targetContainers;0<e.length;){var t=Qh(n.domEventName,n.eventSystemFlags,e[0],n.nativeEvent);if(t!==null)return e=Wo(t),e!==null&&Zh(e),n.blockedOn=t,!1;e.shift()}return!0}function km(n,e,t){ka(n)&&t.delete(e)}function ex(){for(lh=!1;0<Bn.length;){var n=Bn[0];if(n.blockedOn!==null){n=Wo(n.blockedOn),n!==null&&e0(n);break}for(var e=n.targetContainers;0<e.length;){var t=Qh(n.domEventName,n.eventSystemFlags,e[0],n.nativeEvent);if(t!==null){n.blockedOn=t;break}e.shift()}n.blockedOn===null&&Bn.shift()}Mi!==null&&ka(Mi)&&(Mi=null),Ei!==null&&ka(Ei)&&(Ei=null),Ci!==null&&ka(Ci)&&(Ci=null),Lo.forEach(km),Uo.forEach(km)}function no(n,e){n.blockedOn===e&&(n.blockedOn=null,lh||(lh=!0,Ct.unstable_scheduleCallback(Ct.unstable_NormalPriority,ex)))}function i0(n){function e(r){return no(r,n)}if(0<Bn.length){no(Bn[0],n);for(var t=1;t<Bn.length;t++){var i=Bn[t];i.blockedOn===n&&(i.blockedOn=null)}}for(Mi!==null&&no(Mi,n),Ei!==null&&no(Ei,n),Ci!==null&&no(Ci,n),Lo.forEach(e),Uo.forEach(e),t=0;t<eo.length;t++)i=eo[t],i.blockedOn===n&&(i.blockedOn=null);for(;0<eo.length&&(t=eo[0],t.blockedOn===null);)Qy(t),t.blockedOn===null&&eo.shift()}function Ra(n,e){var t={};return t[n.toLowerCase()]=e.toLowerCase(),t["Webkit"+n]="webkit"+e,t["Moz"+n]="moz"+e,t}var Gr={animationend:Ra("Animation","AnimationEnd"),animationiteration:Ra("Animation","AnimationIteration"),animationstart:Ra("Animation","AnimationStart"),transitionend:Ra("Transition","TransitionEnd")},Pu={},r0={};ei&&(r0=document.createElement("div").style,"AnimationEvent"in window||(delete Gr.animationend.animation,delete Gr.animationiteration.animation,delete Gr.animationstart.animation),"TransitionEvent"in window||delete Gr.transitionend.transition);function wl(n){if(Pu[n])return Pu[n];if(!Gr[n])return n;var e=Gr[n],t;for(t in e)if(e.hasOwnProperty(t)&&t in r0)return Pu[n]=e[t];return n}var s0=wl("animationend"),o0=wl("animationiteration"),a0=wl("animationstart"),l0=wl("transitionend"),c0=new Map,Jh=new Map,tx=["abort","abort",s0,"animationEnd",o0,"animationIteration",a0,"animationStart","canplay","canPlay","canplaythrough","canPlayThrough","durationchange","durationChange","emptied","emptied","encrypted","encrypted","ended","ended","error","error","gotpointercapture","gotPointerCapture","load","load","loadeddata","loadedData","loadedmetadata","loadedMetadata","loadstart","loadStart","lostpointercapture","lostPointerCapture","playing","playing","progress","progress","seeking","seeking","stalled","stalled","suspend","suspend","timeupdate","timeUpdate",l0,"transitionEnd","waiting","waiting"];function Kh(n,e){for(var t=0;t<n.length;t+=2){var i=n[t],r=n[t+1];r="on"+(r[0].toUpperCase()+r.slice(1)),Jh.set(i,e),c0.set(i,r),hr(r,[i])}}var nx=Ct.unstable_now;nx();var tt=8;function Vr(n){if(1&n)return tt=15,1;if(2&n)return tt=14,2;if(4&n)return tt=13,4;var e=24&n;return e!==0?(tt=12,e):n&32?(tt=11,32):(e=192&n,e!==0?(tt=10,e):n&256?(tt=9,256):(e=3584&n,e!==0?(tt=8,e):n&4096?(tt=7,4096):(e=4186112&n,e!==0?(tt=6,e):(e=62914560&n,e!==0?(tt=5,e):n&67108864?(tt=4,67108864):n&134217728?(tt=3,134217728):(e=805306368&n,e!==0?(tt=2,e):1073741824&n?(tt=1,1073741824):(tt=8,n))))))}function ix(n){switch(n){case 99:return 15;case 98:return 10;case 97:case 96:return 8;case 95:return 2;default:return 0}}function rx(n){switch(n){case 15:case 14:return 99;case 13:case 12:case 11:case 10:return 98;case 9:case 8:case 7:case 6:case 4:case 5:return 97;case 3:case 2:case 1:return 95;case 0:return 90;default:throw Error(j(358,n))}}function No(n,e){var t=n.pendingLanes;if(t===0)return tt=0;var i=0,r=0,s=n.expiredLanes,o=n.suspendedLanes,a=n.pingedLanes;if(s!==0)i=s,r=tt=15;else if(s=t&134217727,s!==0){var l=s&~o;l!==0?(i=Vr(l),r=tt):(a&=s,a!==0&&(i=Vr(a),r=tt))}else s=t&~o,s!==0?(i=Vr(s),r=tt):a!==0&&(i=Vr(a),r=tt);if(i===0)return 0;if(i=31-Li(i),i=t&((0>i?0:1<<i)<<1)-1,e!==0&&e!==i&&!(e&o)){if(Vr(e),r<=tt)return e;tt=r}if(e=n.entangledLanes,e!==0)for(n=n.entanglements,e&=i;0<e;)t=31-Li(e),r=1<<t,i|=n[t],e&=~r;return i}function u0(n){return n=n.pendingLanes&-1073741825,n!==0?n:n&1073741824?1073741824:0}function Qa(n,e){switch(n){case 15:return 1;case 14:return 2;case 12:return n=Hr(24&~e),n===0?Qa(10,e):n;case 10:return n=Hr(192&~e),n===0?Qa(8,e):n;case 8:return n=Hr(3584&~e),n===0&&(n=Hr(4186112&~e),n===0&&(n=512)),n;case 2:return e=Hr(805306368&~e),e===0&&(e=268435456),e}throw Error(j(358,n))}function Hr(n){return n&-n}function Iu(n){for(var e=[],t=0;31>t;t++)e.push(n);return e}function Sl(n,e,t){n.pendingLanes|=e;var i=e-1;n.suspendedLanes&=i,n.pingedLanes&=i,n=n.eventTimes,e=31-Li(e),n[e]=t}var Li=Math.clz32?Math.clz32:ax,sx=Math.log,ox=Math.LN2;function ax(n){return n===0?32:31-(sx(n)/ox|0)|0}var lx=Ct.unstable_UserBlockingPriority,cx=Ct.unstable_runWithPriority,za=!0;function ux(n,e,t,i){ir||qh();var r=jh,s=ir;ir=!0;try{Jg(r,n,e,t,i)}finally{(ir=s)||$h()}}function hx(n,e,t,i){cx(lx,jh.bind(null,n,e,t,i))}function jh(n,e,t,i){if(za){var r;if((r=(e&4)===0)&&0<Bn.length&&-1<Fm.indexOf(n))n=ch(null,n,e,t,i),Bn.push(n);else{var s=Qh(n,e,t,i);if(s===null)r&&Bm(n,i);else{if(r){if(-1<Fm.indexOf(n)){n=ch(s,n,e,t,i),Bn.push(n);return}if(jy(s,n,e,t,i))return;Bm(n,i)}M0(n,e,i,null,t)}}}}function Qh(n,e,t,i){var r=Xh(i);if(r=rr(r),r!==null){var s=fr(r);if(s===null)r=null;else{var o=s.tag;if(o===13){if(r=jg(s),r!==null)return r;r=null}else if(o===3){if(s.stateNode.hydrate)return s.tag===3?s.stateNode.containerInfo:null;r=null}else s!==r&&(r=null)}}return M0(n,e,i,r,t),null}var xi=null,ed=null,Va=null;function h0(){if(Va)return Va;var n,e=ed,t=e.length,i,r="value"in xi?xi.value:xi.textContent,s=r.length;for(n=0;n<t&&e[n]===r[n];n++);var o=t-n;for(i=1;i<=o&&e[t-i]===r[s-i];i++);return Va=r.slice(n,1<i?1-i:void 0)}function Ha(n){var e=n.keyCode;return"charCode"in n?(n=n.charCode,n===0&&e===13&&(n=13)):n=e,n===10&&(n=13),32<=n||n===13?n:0}function Pa(){return!0}function zm(){return!1}function an(n){function e(t,i,r,s,o){this._reactName=t,this._targetInst=r,this.type=i,this.nativeEvent=s,this.target=o,this.currentTarget=null;for(var a in n)n.hasOwnProperty(a)&&(t=n[a],this[a]=t?t(s):s[a]);return this.isDefaultPrevented=(s.defaultPrevented!=null?s.defaultPrevented:s.returnValue===!1)?Pa:zm,this.isPropagationStopped=zm,this}return st(e.prototype,{preventDefault:function(){this.defaultPrevented=!0;var t=this.nativeEvent;t&&(t.preventDefault?t.preventDefault():typeof t.returnValue!="unknown"&&(t.returnValue=!1),this.isDefaultPrevented=Pa)},stopPropagation:function(){var t=this.nativeEvent;t&&(t.stopPropagation?t.stopPropagation():typeof t.cancelBubble!="unknown"&&(t.cancelBubble=!0),this.isPropagationStopped=Pa)},persist:function(){},isPersistent:Pa}),e}var cs={eventPhase:0,bubbles:0,cancelable:0,timeStamp:function(n){return n.timeStamp||Date.now()},defaultPrevented:0,isTrusted:0},td=an(cs),Go=st({},cs,{view:0,detail:0}),dx=an(Go),Lu,Uu,io,Ml=st({},Go,{screenX:0,screenY:0,clientX:0,clientY:0,pageX:0,pageY:0,ctrlKey:0,shiftKey:0,altKey:0,metaKey:0,getModifierState:nd,button:0,buttons:0,relatedTarget:function(n){return n.relatedTarget===void 0?n.fromElement===n.srcElement?n.toElement:n.fromElement:n.relatedTarget},movementX:function(n){return"movementX"in n?n.movementX:(n!==io&&(io&&n.type==="mousemove"?(Lu=n.screenX-io.screenX,Uu=n.screenY-io.screenY):Uu=Lu=0,io=n),Lu)},movementY:function(n){return"movementY"in n?n.movementY:Uu}}),Vm=an(Ml),fx=st({},Ml,{dataTransfer:0}),px=an(fx),mx=st({},Go,{relatedTarget:0}),Nu=an(mx),gx=st({},cs,{animationName:0,elapsedTime:0,pseudoElement:0}),_x=an(gx),vx=st({},cs,{clipboardData:function(n){return"clipboardData"in n?n.clipboardData:window.clipboardData}}),yx=an(vx),xx=st({},cs,{data:0}),Hm=an(xx),wx={Esc:"Escape",Spacebar:" ",Left:"ArrowLeft",Up:"ArrowUp",Right:"ArrowRight",Down:"ArrowDown",Del:"Delete",Win:"OS",Menu:"ContextMenu",Apps:"ContextMenu",Scroll:"ScrollLock",MozPrintableKey:"Unidentified"},Sx={8:"Backspace",9:"Tab",12:"Clear",13:"Enter",16:"Shift",17:"Control",18:"Alt",19:"Pause",20:"CapsLock",27:"Escape",32:" ",33:"PageUp",34:"PageDown",35:"End",36:"Home",37:"ArrowLeft",38:"ArrowUp",39:"ArrowRight",40:"ArrowDown",45:"Insert",46:"Delete",112:"F1",113:"F2",114:"F3",115:"F4",116:"F5",117:"F6",118:"F7",119:"F8",120:"F9",121:"F10",122:"F11",123:"F12",144:"NumLock",145:"ScrollLock",224:"Meta"},Mx={Alt:"altKey",Control:"ctrlKey",Meta:"metaKey",Shift:"shiftKey"};function Ex(n){var e=this.nativeEvent;return e.getModifierState?e.getModifierState(n):(n=Mx[n])?!!e[n]:!1}function nd(){return Ex}var Cx=st({},Go,{key:function(n){if(n.key){var e=wx[n.key]||n.key;if(e!=="Unidentified")return e}return n.type==="keypress"?(n=Ha(n),n===13?"Enter":String.fromCharCode(n)):n.type==="keydown"||n.type==="keyup"?Sx[n.keyCode]||"Unidentified":""},code:0,location:0,ctrlKey:0,shiftKey:0,altKey:0,metaKey:0,repeat:0,locale:0,getModifierState:nd,charCode:function(n){return n.type==="keypress"?Ha(n):0},keyCode:function(n){return n.type==="keydown"||n.type==="keyup"?n.keyCode:0},which:function(n){return n.type==="keypress"?Ha(n):n.type==="keydown"||n.type==="keyup"?n.keyCode:0}}),Tx=an(Cx),bx=st({},Ml,{pointerId:0,width:0,height:0,pressure:0,tangentialPressure:0,tiltX:0,tiltY:0,twist:0,pointerType:0,isPrimary:0}),Gm=an(bx),Ax=st({},Go,{touches:0,targetTouches:0,changedTouches:0,altKey:0,metaKey:0,ctrlKey:0,shiftKey:0,getModifierState:nd}),Rx=an(Ax),Px=st({},cs,{propertyName:0,elapsedTime:0,pseudoElement:0}),Ix=an(Px),Lx=st({},Ml,{deltaX:function(n){return"deltaX"in n?n.deltaX:"wheelDeltaX"in n?-n.wheelDeltaX:0},deltaY:function(n){return"deltaY"in n?n.deltaY:"wheelDeltaY"in n?-n.wheelDeltaY:"wheelDelta"in n?-n.wheelDelta:0},deltaZ:0,deltaMode:0}),Ux=an(Lx),Nx=[9,13,27,32],id=ei&&"CompositionEvent"in window,wo=null;ei&&"documentMode"in document&&(wo=document.documentMode);var Dx=ei&&"TextEvent"in window&&!wo,d0=ei&&(!id||wo&&8<wo&&11>=wo),Wm=" ",Xm=!1;function f0(n,e){switch(n){case"keyup":return Nx.indexOf(e.keyCode)!==-1;case"keydown":return e.keyCode!==229;case"keypress":case"mousedown":case"focusout":return!0;default:return!1}}function p0(n){return n=n.detail,typeof n=="object"&&"data"in n?n.data:null}var Wr=!1;function Ox(n,e){switch(n){case"compositionend":return p0(e);case"keypress":return e.which!==32?null:(Xm=!0,Wm);case"textInput":return n=e.data,n===Wm&&Xm?null:n;default:return null}}function Fx(n,e){if(Wr)return n==="compositionend"||!id&&f0(n,e)?(n=h0(),Va=ed=xi=null,Wr=!1,n):null;switch(n){case"paste":return null;case"keypress":if(!(e.ctrlKey||e.altKey||e.metaKey)||e.ctrlKey&&e.altKey){if(e.char&&1<e.char.length)return e.char;if(e.which)return String.fromCharCode(e.which)}return null;case"compositionend":return d0&&e.locale!=="ko"?null:e.data;default:return null}}var Bx={color:!0,date:!0,datetime:!0,"datetime-local":!0,email:!0,month:!0,number:!0,password:!0,range:!0,search:!0,tel:!0,text:!0,time:!0,url:!0,week:!0};function Ym(n){var e=n&&n.nodeName&&n.nodeName.toLowerCase();return e==="input"?!!Bx[n.type]:e==="textarea"}function m0(n,e,t,i){$g(i),e=el(e,"onChange"),0<e.length&&(t=new td("onChange","change",null,t,i),n.push({event:t,listeners:e}))}var So=null,Do=null;function kx(n){x0(n,0)}function El(n){var e=Yr(n);if(Vg(e))return n}function zx(n,e){if(n==="change")return e}var g0=!1;ei&&(ei?(La="oninput"in document,La||(Du=document.createElement("div"),Du.setAttribute("oninput","return;"),La=typeof Du.oninput=="function"),Ia=La):Ia=!1,g0=Ia&&(!document.documentMode||9<document.documentMode));var Ia,La,Du;function qm(){So&&(So.detachEvent("onpropertychange",_0),Do=So=null)}function _0(n){if(n.propertyName==="value"&&El(Do)){var e=[];if(m0(e,Do,n,Xh(n)),n=kx,ir)n(e);else{ir=!0;try{Yh(n,e)}finally{ir=!1,$h()}}}}function Vx(n,e,t){n==="focusin"?(qm(),So=e,Do=t,So.attachEvent("onpropertychange",_0)):n==="focusout"&&qm()}function Hx(n){if(n==="selectionchange"||n==="keyup"||n==="keydown")return El(Do)}function Gx(n,e){if(n==="click")return El(e)}function Wx(n,e){if(n==="input"||n==="change")return El(e)}function Xx(n,e){return n===e&&(n!==0||1/n===1/e)||n!==n&&e!==e}var mn=typeof Object.is=="function"?Object.is:Xx,Yx=Object.prototype.hasOwnProperty;function Oo(n,e){if(mn(n,e))return!0;if(typeof n!="object"||n===null||typeof e!="object"||e===null)return!1;var t=Object.keys(n),i=Object.keys(e);if(t.length!==i.length)return!1;for(i=0;i<t.length;i++)if(!Yx.call(e,t[i])||!mn(n[t[i]],e[t[i]]))return!1;return!0}function $m(n){for(;n&&n.firstChild;)n=n.firstChild;return n}function Zm(n,e){var t=$m(n);n=0;for(var i;t;){if(t.nodeType===3){if(i=n+t.textContent.length,n<=e&&i>=e)return{node:t,offset:e-n};n=i}e:{for(;t;){if(t.nextSibling){t=t.nextSibling;break e}t=t.parentNode}t=void 0}t=$m(t)}}function v0(n,e){return n&&e?n===e?!0:n&&n.nodeType===3?!1:e&&e.nodeType===3?v0(n,e.parentNode):"contains"in n?n.contains(e):n.compareDocumentPosition?!!(n.compareDocumentPosition(e)&16):!1:!1}function Jm(){for(var n=window,e=Ja();e instanceof n.HTMLIFrameElement;){try{var t=typeof e.contentWindow.location.href=="string"}catch{t=!1}if(t)n=e.contentWindow;else break;e=Ja(n.document)}return e}function uh(n){var e=n&&n.nodeName&&n.nodeName.toLowerCase();return e&&(e==="input"&&(n.type==="text"||n.type==="search"||n.type==="tel"||n.type==="url"||n.type==="password")||e==="textarea"||n.contentEditable==="true")}var qx=ei&&"documentMode"in document&&11>=document.documentMode,Xr=null,hh=null,Mo=null,dh=!1;function Km(n,e,t){var i=t.window===t?t.document:t.nodeType===9?t:t.ownerDocument;dh||Xr==null||Xr!==Ja(i)||(i=Xr,"selectionStart"in i&&uh(i)?i={start:i.selectionStart,end:i.selectionEnd}:(i=(i.ownerDocument&&i.ownerDocument.defaultView||window).getSelection(),i={anchorNode:i.anchorNode,anchorOffset:i.anchorOffset,focusNode:i.focusNode,focusOffset:i.focusOffset}),Mo&&Oo(Mo,i)||(Mo=i,i=el(hh,"onSelect"),0<i.length&&(e=new td("onSelect","select",null,e,t),n.push({event:e,listeners:i}),e.target=Xr)))}Kh("cancel cancel click click close close contextmenu contextMenu copy copy cut cut auxclick auxClick dblclick doubleClick dragend dragEnd dragstart dragStart drop drop focusin focus focusout blur input input invalid invalid keydown keyDown keypress keyPress keyup keyUp mousedown mouseDown mouseup mouseUp paste paste pause pause play play pointercancel pointerCancel pointerdown pointerDown pointerup pointerUp ratechange rateChange reset reset seeked seeked submit submit touchcancel touchCancel touchend touchEnd touchstart touchStart volumechange volumeChange".split(" "),0);Kh("drag drag dragenter dragEnter dragexit dragExit dragleave dragLeave dragover dragOver mousemove mouseMove mouseout mouseOut mouseover mouseOver pointermove pointerMove pointerout pointerOut pointerover pointerOver scroll scroll toggle toggle touchmove touchMove wheel wheel".split(" "),1);Kh(tx,2);for(Ou="change selectionchange textInput compositionstart compositionend compositionupdate".split(" "),Ua=0;Ua<Ou.length;Ua++)Jh.set(Ou[Ua],0);var Ou,Ua;ss("onMouseEnter",["mouseout","mouseover"]);ss("onMouseLeave",["mouseout","mouseover"]);ss("onPointerEnter",["pointerout","pointerover"]);ss("onPointerLeave",["pointerout","pointerover"]);hr("onChange","change click focusin focusout input keydown keyup selectionchange".split(" "));hr("onSelect","focusout contextmenu dragend focusin keydown keyup mousedown mouseup selectionchange".split(" "));hr("onBeforeInput",["compositionend","keypress","textInput","paste"]);hr("onCompositionEnd","compositionend focusout keydown keypress keyup mousedown".split(" "));hr("onCompositionStart","compositionstart focusout keydown keypress keyup mousedown".split(" "));hr("onCompositionUpdate","compositionupdate focusout keydown keypress keyup mousedown".split(" "));var po="abort canplay canplaythrough durationchange emptied encrypted ended error loadeddata loadedmetadata loadstart pause play playing progress ratechange seeked seeking stalled suspend timeupdate volumechange waiting".split(" "),y0=new Set("cancel close invalid load scroll toggle".split(" ").concat(po));function jm(n,e,t){var i=n.type||"unknown-event";n.currentTarget=t,Jy(i,e,void 0,n),n.currentTarget=null}function x0(n,e){e=(e&4)!==0;for(var t=0;t<n.length;t++){var i=n[t],r=i.event;i=i.listeners;e:{var s=void 0;if(e)for(var o=i.length-1;0<=o;o--){var a=i[o],l=a.instance,c=a.currentTarget;if(a=a.listener,l!==s&&r.isPropagationStopped())break e;jm(r,a,c),s=l}else for(o=0;o<i.length;o++){if(a=i[o],l=a.instance,c=a.currentTarget,a=a.listener,l!==s&&r.isPropagationStopped())break e;jm(r,a,c),s=l}}}if(ja)throw n=ah,ja=!1,ah=null,n}function nt(n,e){var t=C0(e),i=n+"__bubble";t.has(i)||(S0(e,n,2,!1),t.add(i))}var Qm="_reactListening"+Math.random().toString(36).slice(2);function w0(n){n[Qm]||(n[Qm]=!0,Bg.forEach(function(e){y0.has(e)||eg(e,!1,n,null),eg(e,!0,n,null)}))}function eg(n,e,t,i){var r=4<arguments.length&&arguments[4]!==void 0?arguments[4]:0,s=t;if(n==="selectionchange"&&t.nodeType!==9&&(s=t.ownerDocument),i!==null&&!e&&y0.has(n)){if(n!=="scroll")return;r|=2,s=i}var o=C0(s),a=n+"__"+(e?"capture":"bubble");o.has(a)||(e&&(r|=4),S0(s,n,r,e),o.add(a))}function S0(n,e,t,i){var r=Jh.get(e);switch(r===void 0?2:r){case 0:r=ux;break;case 1:r=hx;break;default:r=jh}t=r.bind(null,e,t,n),r=void 0,!oh||e!=="touchstart"&&e!=="touchmove"&&e!=="wheel"||(r=!0),i?r!==void 0?n.addEventListener(e,t,{capture:!0,passive:r}):n.addEventListener(e,t,!0):r!==void 0?n.addEventListener(e,t,{passive:r}):n.addEventListener(e,t,!1)}function M0(n,e,t,i,r){var s=i;if(!(e&1)&&!(e&2)&&i!==null)e:for(;;){if(i===null)return;var o=i.tag;if(o===3||o===4){var a=i.stateNode.containerInfo;if(a===r||a.nodeType===8&&a.parentNode===r)break;if(o===4)for(o=i.return;o!==null;){var l=o.tag;if((l===3||l===4)&&(l=o.stateNode.containerInfo,l===r||l.nodeType===8&&l.parentNode===r))return;o=o.return}for(;a!==null;){if(o=rr(a),o===null)return;if(l=o.tag,l===5||l===6){i=s=o;continue e}a=a.parentNode}}i=i.return}Yy(function(){var c=s,u=Xh(t),p=[];e:{var f=c0.get(n);if(f!==void 0){var m=td,_=n;switch(n){case"keypress":if(Ha(t)===0)break e;case"keydown":case"keyup":m=Tx;break;case"focusin":_="focus",m=Nu;break;case"focusout":_="blur",m=Nu;break;case"beforeblur":case"afterblur":m=Nu;break;case"click":if(t.button===2)break e;case"auxclick":case"dblclick":case"mousedown":case"mousemove":case"mouseup":case"mouseout":case"mouseover":case"contextmenu":m=Vm;break;case"drag":case"dragend":case"dragenter":case"dragexit":case"dragleave":case"dragover":case"dragstart":case"drop":m=px;break;case"touchcancel":case"touchend":case"touchmove":case"touchstart":m=Rx;break;case s0:case o0:case a0:m=_x;break;case l0:m=Ix;break;case"scroll":m=dx;break;case"wheel":m=Ux;break;case"copy":case"cut":case"paste":m=yx;break;case"gotpointercapture":case"lostpointercapture":case"pointercancel":case"pointerdown":case"pointermove":case"pointerout":case"pointerover":case"pointerup":m=Gm}var y=(e&4)!==0,d=!y&&n==="scroll",h=y?f!==null?f+"Capture":null:f;y=[];for(var g=c,v;g!==null;){v=g;var x=v.stateNode;if(v.tag===5&&x!==null&&(v=x,h!==null&&(x=Io(g,h),x!=null&&y.push(Fo(g,x,v)))),d)break;g=g.return}0<y.length&&(f=new m(f,_,null,t,u),p.push({event:f,listeners:y}))}}if(!(e&7)){e:{if(f=n==="mouseover"||n==="pointerover",m=n==="mouseout"||n==="pointerout",f&&!(e&16)&&(_=t.relatedTarget||t.fromElement)&&(rr(_)||_[us]))break e;if((m||f)&&(f=u.window===u?u:(f=u.ownerDocument)?f.defaultView||f.parentWindow:window,m?(_=t.relatedTarget||t.toElement,m=c,_=_?rr(_):null,_!==null&&(d=fr(_),_!==d||_.tag!==5&&_.tag!==6)&&(_=null)):(m=null,_=c),m!==_)){if(y=Vm,x="onMouseLeave",h="onMouseEnter",g="mouse",(n==="pointerout"||n==="pointerover")&&(y=Gm,x="onPointerLeave",h="onPointerEnter",g="pointer"),d=m==null?f:Yr(m),v=_==null?f:Yr(_),f=new y(x,g+"leave",m,t,u),f.target=d,f.relatedTarget=v,x=null,rr(u)===c&&(y=new y(h,g+"enter",_,t,u),y.target=v,y.relatedTarget=d,x=y),d=x,m&&_)t:{for(y=m,h=_,g=0,v=y;v;v=zr(v))g++;for(v=0,x=h;x;x=zr(x))v++;for(;0<g-v;)y=zr(y),g--;for(;0<v-g;)h=zr(h),v--;for(;g--;){if(y===h||h!==null&&y===h.alternate)break t;y=zr(y),h=zr(h)}y=null}else y=null;m!==null&&tg(p,f,m,y,!1),_!==null&&d!==null&&tg(p,d,_,y,!0)}}e:{if(f=c?Yr(c):window,m=f.nodeName&&f.nodeName.toLowerCase(),m==="select"||m==="input"&&f.type==="file")var R=zx;else if(Ym(f))if(g0)R=Wx;else{R=Hx;var C=Vx}else(m=f.nodeName)&&m.toLowerCase()==="input"&&(f.type==="checkbox"||f.type==="radio")&&(R=Gx);if(R&&(R=R(n,c))){m0(p,R,t,u);break e}C&&C(n,f,c),n==="focusout"&&(C=f._wrapperState)&&C.controlled&&f.type==="number"&&ju(f,"number",f.value)}switch(C=c?Yr(c):window,n){case"focusin":(Ym(C)||C.contentEditable==="true")&&(Xr=C,hh=c,Mo=null);break;case"focusout":Mo=hh=Xr=null;break;case"mousedown":dh=!0;break;case"contextmenu":case"mouseup":case"dragend":dh=!1,Km(p,t,u);break;case"selectionchange":if(qx)break;case"keydown":case"keyup":Km(p,t,u)}var T;if(id)e:{switch(n){case"compositionstart":var L="onCompositionStart";break e;case"compositionend":L="onCompositionEnd";break e;case"compositionupdate":L="onCompositionUpdate";break e}L=void 0}else Wr?f0(n,t)&&(L="onCompositionEnd"):n==="keydown"&&t.keyCode===229&&(L="onCompositionStart");L&&(d0&&t.locale!=="ko"&&(Wr||L!=="onCompositionStart"?L==="onCompositionEnd"&&Wr&&(T=h0()):(xi=u,ed="value"in xi?xi.value:xi.textContent,Wr=!0)),C=el(c,L),0<C.length&&(L=new Hm(L,n,null,t,u),p.push({event:L,listeners:C}),T?L.data=T:(T=p0(t),T!==null&&(L.data=T)))),(T=Dx?Ox(n,t):Fx(n,t))&&(c=el(c,"onBeforeInput"),0<c.length&&(u=new Hm("onBeforeInput","beforeinput",null,t,u),p.push({event:u,listeners:c}),u.data=T))}x0(p,e)})}function Fo(n,e,t){return{instance:n,listener:e,currentTarget:t}}function el(n,e){for(var t=e+"Capture",i=[];n!==null;){var r=n,s=r.stateNode;r.tag===5&&s!==null&&(r=s,s=Io(n,t),s!=null&&i.unshift(Fo(n,s,r)),s=Io(n,e),s!=null&&i.push(Fo(n,s,r))),n=n.return}return i}function zr(n){if(n===null)return null;do n=n.return;while(n&&n.tag!==5);return n||null}function tg(n,e,t,i,r){for(var s=e._reactName,o=[];t!==null&&t!==i;){var a=t,l=a.alternate,c=a.stateNode;if(l!==null&&l===i)break;a.tag===5&&c!==null&&(a=c,r?(l=Io(t,s),l!=null&&o.unshift(Fo(t,l,a))):r||(l=Io(t,s),l!=null&&o.push(Fo(t,l,a)))),t=t.return}o.length!==0&&n.push({event:e,listeners:o})}function tl(){}var Fu=null,Bu=null;function E0(n,e){switch(n){case"button":case"input":case"select":case"textarea":return!!e.autoFocus}return!1}function fh(n,e){return n==="textarea"||n==="option"||n==="noscript"||typeof e.children=="string"||typeof e.children=="number"||typeof e.dangerouslySetInnerHTML=="object"&&e.dangerouslySetInnerHTML!==null&&e.dangerouslySetInnerHTML.__html!=null}var ng=typeof setTimeout=="function"?setTimeout:void 0,$x=typeof clearTimeout=="function"?clearTimeout:void 0;function rd(n){n.nodeType===1?n.textContent="":n.nodeType===9&&(n=n.body,n!=null&&(n.textContent=""))}function Qr(n){for(;n!=null;n=n.nextSibling){var e=n.nodeType;if(e===1||e===3)break}return n}function ig(n){n=n.previousSibling;for(var e=0;n;){if(n.nodeType===8){var t=n.data;if(t==="$"||t==="$!"||t==="$?"){if(e===0)return n;e--}else t==="/$"&&e++}n=n.previousSibling}return null}var ku=0;function Zx(n){return{$$typeof:Gh,toString:n,valueOf:n}}var Cl=Math.random().toString(36).slice(2),wi="__reactFiber$"+Cl,nl="__reactProps$"+Cl,us="__reactContainer$"+Cl,rg="__reactEvents$"+Cl;function rr(n){var e=n[wi];if(e)return e;for(var t=n.parentNode;t;){if(e=t[us]||t[wi]){if(t=e.alternate,e.child!==null||t!==null&&t.child!==null)for(n=ig(n);n!==null;){if(t=n[wi])return t;n=ig(n)}return e}n=t,t=n.parentNode}return null}function Wo(n){return n=n[wi]||n[us],!n||n.tag!==5&&n.tag!==6&&n.tag!==13&&n.tag!==3?null:n}function Yr(n){if(n.tag===5||n.tag===6)return n.stateNode;throw Error(j(33))}function Tl(n){return n[nl]||null}function C0(n){var e=n[rg];return e===void 0&&(e=n[rg]=new Set),e}var ph=[],qr=-1;function Oi(n){return{current:n}}function it(n){0>qr||(n.current=ph[qr],ph[qr]=null,qr--)}function dt(n,e){qr++,ph[qr]=n.current,n.current=e}var Ui={},Bt=Oi(Ui),Kt=Oi(!1),lr=Ui;function os(n,e){var t=n.type.contextTypes;if(!t)return Ui;var i=n.stateNode;if(i&&i.__reactInternalMemoizedUnmaskedChildContext===e)return i.__reactInternalMemoizedMaskedChildContext;var r={},s;for(s in t)r[s]=e[s];return i&&(n=n.stateNode,n.__reactInternalMemoizedUnmaskedChildContext=e,n.__reactInternalMemoizedMaskedChildContext=r),r}function jt(n){return n=n.childContextTypes,n!=null}function il(){it(Kt),it(Bt)}function sg(n,e,t){if(Bt.current!==Ui)throw Error(j(168));dt(Bt,e),dt(Kt,t)}function T0(n,e,t){var i=n.stateNode;if(n=e.childContextTypes,typeof i.getChildContext!="function")return t;i=i.getChildContext();for(var r in i)if(!(r in n))throw Error(j(108,Zr(e)||"Unknown",r));return st({},t,i)}function Ga(n){return n=(n=n.stateNode)&&n.__reactInternalMemoizedMergedChildContext||Ui,lr=Bt.current,dt(Bt,n),dt(Kt,Kt.current),!0}function og(n,e,t){var i=n.stateNode;if(!i)throw Error(j(169));t?(n=T0(n,e,lr),i.__reactInternalMemoizedMergedChildContext=n,it(Kt),it(Bt),dt(Bt,n)):it(Kt),dt(Kt,t)}var sd=null,ar=null,Jx=Ct.unstable_runWithPriority,od=Ct.unstable_scheduleCallback,mh=Ct.unstable_cancelCallback,Kx=Ct.unstable_shouldYield,ag=Ct.unstable_requestPaint,gh=Ct.unstable_now,jx=Ct.unstable_getCurrentPriorityLevel,bl=Ct.unstable_ImmediatePriority,b0=Ct.unstable_UserBlockingPriority,A0=Ct.unstable_NormalPriority,R0=Ct.unstable_LowPriority,P0=Ct.unstable_IdlePriority,zu={},Qx=ag!==void 0?ag:function(){},Zn=null,Wa=null,Vu=!1,lg=gh(),Ot=1e4>lg?gh:function(){return gh()-lg};function as(){switch(jx()){case bl:return 99;case b0:return 98;case A0:return 97;case R0:return 96;case P0:return 95;default:throw Error(j(332))}}function I0(n){switch(n){case 99:return bl;case 98:return b0;case 97:return A0;case 96:return R0;case 95:return P0;default:throw Error(j(332))}}function cr(n,e){return n=I0(n),Jx(n,e)}function Bo(n,e,t){return n=I0(n),od(n,e,t)}function Gn(){if(Wa!==null){var n=Wa;Wa=null,mh(n)}L0()}function L0(){if(!Vu&&Zn!==null){Vu=!0;var n=0;try{var e=Zn;cr(99,function(){for(;n<e.length;n++){var t=e[n];do t=t(!0);while(t!==null)}}),Zn=null}catch(t){throw Zn!==null&&(Zn=Zn.slice(n+1)),od(bl,Gn),t}finally{Vu=!1}}}var ew=dr.ReactCurrentBatchConfig;function bn(n,e){if(n&&n.defaultProps){e=st({},e),n=n.defaultProps;for(var t in n)e[t]===void 0&&(e[t]=n[t]);return e}return e}var rl=Oi(null),sl=null,$r=null,ol=null;function ad(){ol=$r=sl=null}function ld(n){var e=rl.current;it(rl),n.type._context._currentValue=e}function U0(n,e){for(;n!==null;){var t=n.alternate;if((n.childLanes&e)===e){if(t===null||(t.childLanes&e)===e)break;t.childLanes|=e}else n.childLanes|=e,t!==null&&(t.childLanes|=e);n=n.return}}function es(n,e){sl=n,ol=$r=null,n=n.dependencies,n!==null&&n.firstContext!==null&&(n.lanes&e&&(An=!0),n.firstContext=null)}function vn(n,e){if(ol!==n&&e!==!1&&e!==0)if((typeof e!="number"||e===1073741823)&&(ol=n,e=1073741823),e={context:n,observedBits:e,next:null},$r===null){if(sl===null)throw Error(j(308));$r=e,sl.dependencies={lanes:0,firstContext:e,responders:null}}else $r=$r.next=e;return n._currentValue}var vi=!1;function cd(n){n.updateQueue={baseState:n.memoizedState,firstBaseUpdate:null,lastBaseUpdate:null,shared:{pending:null},effects:null}}function N0(n,e){n=n.updateQueue,e.updateQueue===n&&(e.updateQueue={baseState:n.baseState,firstBaseUpdate:n.firstBaseUpdate,lastBaseUpdate:n.lastBaseUpdate,shared:n.shared,effects:n.effects})}function Ti(n,e){return{eventTime:n,lane:e,tag:0,payload:null,callback:null,next:null}}function bi(n,e){if(n=n.updateQueue,n!==null){n=n.shared;var t=n.pending;t===null?e.next=e:(e.next=t.next,t.next=e),n.pending=e}}function cg(n,e){var t=n.updateQueue,i=n.alternate;if(i!==null&&(i=i.updateQueue,t===i)){var r=null,s=null;if(t=t.firstBaseUpdate,t!==null){do{var o={eventTime:t.eventTime,lane:t.lane,tag:t.tag,payload:t.payload,callback:t.callback,next:null};s===null?r=s=o:s=s.next=o,t=t.next}while(t!==null);s===null?r=s=e:s=s.next=e}else r=s=e;t={baseState:i.baseState,firstBaseUpdate:r,lastBaseUpdate:s,shared:i.shared,effects:i.effects},n.updateQueue=t;return}n=t.lastBaseUpdate,n===null?t.firstBaseUpdate=e:n.next=e,t.lastBaseUpdate=e}function ko(n,e,t,i){var r=n.updateQueue;vi=!1;var s=r.firstBaseUpdate,o=r.lastBaseUpdate,a=r.shared.pending;if(a!==null){r.shared.pending=null;var l=a,c=l.next;l.next=null,o===null?s=c:o.next=c,o=l;var u=n.alternate;if(u!==null){u=u.updateQueue;var p=u.lastBaseUpdate;p!==o&&(p===null?u.firstBaseUpdate=c:p.next=c,u.lastBaseUpdate=l)}}if(s!==null){p=r.baseState,o=0,u=c=l=null;do{a=s.lane;var f=s.eventTime;if((i&a)===a){u!==null&&(u=u.next={eventTime:f,lane:0,tag:s.tag,payload:s.payload,callback:s.callback,next:null});e:{var m=n,_=s;switch(a=e,f=t,_.tag){case 1:if(m=_.payload,typeof m=="function"){p=m.call(f,p,a);break e}p=m;break e;case 3:m.flags=m.flags&-4097|64;case 0:if(m=_.payload,a=typeof m=="function"?m.call(f,p,a):m,a==null)break e;p=st({},p,a);break e;case 2:vi=!0}}s.callback!==null&&(n.flags|=32,a=r.effects,a===null?r.effects=[s]:a.push(s))}else f={eventTime:f,lane:a,tag:s.tag,payload:s.payload,callback:s.callback,next:null},u===null?(c=u=f,l=p):u=u.next=f,o|=a;if(s=s.next,s===null){if(a=r.shared.pending,a===null)break;s=a.next,a.next=null,r.lastBaseUpdate=a,r.shared.pending=null}}while(!0);u===null&&(l=p),r.baseState=l,r.firstBaseUpdate=c,r.lastBaseUpdate=u,Yo|=o,n.lanes=o,n.memoizedState=p}}function ug(n,e,t){if(n=e.effects,e.effects=null,n!==null)for(e=0;e<n.length;e++){var i=n[e],r=i.callback;if(r!==null){if(i.callback=null,i=t,typeof r!="function")throw Error(j(191,r));r.call(i)}}}var D0=new vl.Component().refs;function al(n,e,t,i){e=n.memoizedState,t=t(i,e),t=t==null?e:st({},e,t),n.memoizedState=t,n.lanes===0&&(n.updateQueue.baseState=t)}var Al={isMounted:function(n){return(n=n._reactInternals)?fr(n)===n:!1},enqueueSetState:function(n,e,t){n=n._reactInternals;var i=on(),r=Ai(n),s=Ti(i,r);s.payload=e,t!=null&&(s.callback=t),bi(n,s),Ri(n,r,i)},enqueueReplaceState:function(n,e,t){n=n._reactInternals;var i=on(),r=Ai(n),s=Ti(i,r);s.tag=1,s.payload=e,t!=null&&(s.callback=t),bi(n,s),Ri(n,r,i)},enqueueForceUpdate:function(n,e){n=n._reactInternals;var t=on(),i=Ai(n),r=Ti(t,i);r.tag=2,e!=null&&(r.callback=e),bi(n,r),Ri(n,i,t)}};function hg(n,e,t,i,r,s,o){return n=n.stateNode,typeof n.shouldComponentUpdate=="function"?n.shouldComponentUpdate(i,s,o):e.prototype&&e.prototype.isPureReactComponent?!Oo(t,i)||!Oo(r,s):!0}function O0(n,e,t){var i=!1,r=Ui,s=e.contextType;return typeof s=="object"&&s!==null?s=vn(s):(r=jt(e)?lr:Bt.current,i=e.contextTypes,s=(i=i!=null)?os(n,r):Ui),e=new e(t,s),n.memoizedState=e.state!==null&&e.state!==void 0?e.state:null,e.updater=Al,n.stateNode=e,e._reactInternals=n,i&&(n=n.stateNode,n.__reactInternalMemoizedUnmaskedChildContext=r,n.__reactInternalMemoizedMaskedChildContext=s),e}function dg(n,e,t,i){n=e.state,typeof e.componentWillReceiveProps=="function"&&e.componentWillReceiveProps(t,i),typeof e.UNSAFE_componentWillReceiveProps=="function"&&e.UNSAFE_componentWillReceiveProps(t,i),e.state!==n&&Al.enqueueReplaceState(e,e.state,null)}function _h(n,e,t,i){var r=n.stateNode;r.props=t,r.state=n.memoizedState,r.refs=D0,cd(n);var s=e.contextType;typeof s=="object"&&s!==null?r.context=vn(s):(s=jt(e)?lr:Bt.current,r.context=os(n,s)),ko(n,t,r,i),r.state=n.memoizedState,s=e.getDerivedStateFromProps,typeof s=="function"&&(al(n,e,s,t),r.state=n.memoizedState),typeof e.getDerivedStateFromProps=="function"||typeof r.getSnapshotBeforeUpdate=="function"||typeof r.UNSAFE_componentWillMount!="function"&&typeof r.componentWillMount!="function"||(e=r.state,typeof r.componentWillMount=="function"&&r.componentWillMount(),typeof r.UNSAFE_componentWillMount=="function"&&r.UNSAFE_componentWillMount(),e!==r.state&&Al.enqueueReplaceState(r,r.state,null),ko(n,t,r,i),r.state=n.memoizedState),typeof r.componentDidMount=="function"&&(n.flags|=4)}var Na=Array.isArray;function ro(n,e,t){if(n=t.ref,n!==null&&typeof n!="function"&&typeof n!="object"){if(t._owner){if(t=t._owner,t){if(t.tag!==1)throw Error(j(309));var i=t.stateNode}if(!i)throw Error(j(147,n));var r=""+n;return e!==null&&e.ref!==null&&typeof e.ref=="function"&&e.ref._stringRef===r?e.ref:(e=function(s){var o=i.refs;o===D0&&(o=i.refs={}),s===null?delete o[r]:o[r]=s},e._stringRef=r,e)}if(typeof n!="string")throw Error(j(284));if(!t._owner)throw Error(j(290,n))}return n}function Da(n,e){if(n.type!=="textarea")throw Error(j(31,Object.prototype.toString.call(e)==="[object Object]"?"object with keys {"+Object.keys(e).join(", ")+"}":e))}function F0(n){function e(d,h){if(n){var g=d.lastEffect;g!==null?(g.nextEffect=h,d.lastEffect=h):d.firstEffect=d.lastEffect=h,h.nextEffect=null,h.flags=8}}function t(d,h){if(!n)return null;for(;h!==null;)e(d,h),h=h.sibling;return null}function i(d,h){for(d=new Map;h!==null;)h.key!==null?d.set(h.key,h):d.set(h.index,h),h=h.sibling;return d}function r(d,h){return d=Di(d,h),d.index=0,d.sibling=null,d}function s(d,h,g){return d.index=g,n?(g=d.alternate,g!==null?(g=g.index,g<h?(d.flags=2,h):g):(d.flags=2,h)):h}function o(d){return n&&d.alternate===null&&(d.flags=2),d}function a(d,h,g,v){return h===null||h.tag!==6?(h=Yu(g,d.mode,v),h.return=d,h):(h=r(h,g),h.return=d,h)}function l(d,h,g,v){return h!==null&&h.elementType===g.type?(v=r(h,g.props),v.ref=ro(d,h,g),v.return=d,v):(v=$a(g.type,g.key,g.props,null,d.mode,v),v.ref=ro(d,h,g),v.return=d,v)}function c(d,h,g,v){return h===null||h.tag!==4||h.stateNode.containerInfo!==g.containerInfo||h.stateNode.implementation!==g.implementation?(h=qu(g,d.mode,v),h.return=d,h):(h=r(h,g.children||[]),h.return=d,h)}function u(d,h,g,v,x){return h===null||h.tag!==7?(h=rs(g,d.mode,v,x),h.return=d,h):(h=r(h,g),h.return=d,h)}function p(d,h,g){if(typeof h=="string"||typeof h=="number")return h=Yu(""+h,d.mode,g),h.return=d,h;if(typeof h=="object"&&h!==null){switch(h.$$typeof){case ho:return g=$a(h.type,h.key,h.props,null,d.mode,g),g.ref=ro(d,null,h),g.return=d,g;case nr:return h=qu(h,d.mode,g),h.return=d,h}if(Na(h)||Qs(h))return h=rs(h,d.mode,g,null),h.return=d,h;Da(d,h)}return null}function f(d,h,g,v){var x=h!==null?h.key:null;if(typeof g=="string"||typeof g=="number")return x!==null?null:a(d,h,""+g,v);if(typeof g=="object"&&g!==null){switch(g.$$typeof){case ho:return g.key===x?g.type===yi?u(d,h,g.props.children,v,x):l(d,h,g,v):null;case nr:return g.key===x?c(d,h,g,v):null}if(Na(g)||Qs(g))return x!==null?null:u(d,h,g,v,null);Da(d,g)}return null}function m(d,h,g,v,x){if(typeof v=="string"||typeof v=="number")return d=d.get(g)||null,a(h,d,""+v,x);if(typeof v=="object"&&v!==null){switch(v.$$typeof){case ho:return d=d.get(v.key===null?g:v.key)||null,v.type===yi?u(h,d,v.props.children,x,v.key):l(h,d,v,x);case nr:return d=d.get(v.key===null?g:v.key)||null,c(h,d,v,x)}if(Na(v)||Qs(v))return d=d.get(g)||null,u(h,d,v,x,null);Da(h,v)}return null}function _(d,h,g,v){for(var x=null,R=null,C=h,T=h=0,L=null;C!==null&&T<g.length;T++){C.index>T?(L=C,C=null):L=C.sibling;var E=f(d,C,g[T],v);if(E===null){C===null&&(C=L);break}n&&C&&E.alternate===null&&e(d,C),h=s(E,h,T),R===null?x=E:R.sibling=E,R=E,C=L}if(T===g.length)return t(d,C),x;if(C===null){for(;T<g.length;T++)C=p(d,g[T],v),C!==null&&(h=s(C,h,T),R===null?x=C:R.sibling=C,R=C);return x}for(C=i(d,C);T<g.length;T++)L=m(C,d,T,g[T],v),L!==null&&(n&&L.alternate!==null&&C.delete(L.key===null?T:L.key),h=s(L,h,T),R===null?x=L:R.sibling=L,R=L);return n&&C.forEach(function(M){return e(d,M)}),x}function y(d,h,g,v){var x=Qs(g);if(typeof x!="function")throw Error(j(150));if(g=x.call(g),g==null)throw Error(j(151));for(var R=x=null,C=h,T=h=0,L=null,E=g.next();C!==null&&!E.done;T++,E=g.next()){C.index>T?(L=C,C=null):L=C.sibling;var M=f(d,C,E.value,v);if(M===null){C===null&&(C=L);break}n&&C&&M.alternate===null&&e(d,C),h=s(M,h,T),R===null?x=M:R.sibling=M,R=M,C=L}if(E.done)return t(d,C),x;if(C===null){for(;!E.done;T++,E=g.next())E=p(d,E.value,v),E!==null&&(h=s(E,h,T),R===null?x=E:R.sibling=E,R=E);return x}for(C=i(d,C);!E.done;T++,E=g.next())E=m(C,d,T,E.value,v),E!==null&&(n&&E.alternate!==null&&C.delete(E.key===null?T:E.key),h=s(E,h,T),R===null?x=E:R.sibling=E,R=E);return n&&C.forEach(function(P){return e(d,P)}),x}return function(d,h,g,v){var x=typeof g=="object"&&g!==null&&g.type===yi&&g.key===null;x&&(g=g.props.children);var R=typeof g=="object"&&g!==null;if(R)switch(g.$$typeof){case ho:e:{for(R=g.key,x=h;x!==null;){if(x.key===R){switch(x.tag){case 7:if(g.type===yi){t(d,x.sibling),h=r(x,g.props.children),h.return=d,d=h;break e}break;default:if(x.elementType===g.type){t(d,x.sibling),h=r(x,g.props),h.ref=ro(d,x,g),h.return=d,d=h;break e}}t(d,x);break}else e(d,x);x=x.sibling}g.type===yi?(h=rs(g.props.children,d.mode,v,g.key),h.return=d,d=h):(v=$a(g.type,g.key,g.props,null,d.mode,v),v.ref=ro(d,h,g),v.return=d,d=v)}return o(d);case nr:e:{for(x=g.key;h!==null;){if(h.key===x)if(h.tag===4&&h.stateNode.containerInfo===g.containerInfo&&h.stateNode.implementation===g.implementation){t(d,h.sibling),h=r(h,g.children||[]),h.return=d,d=h;break e}else{t(d,h);break}else e(d,h);h=h.sibling}h=qu(g,d.mode,v),h.return=d,d=h}return o(d)}if(typeof g=="string"||typeof g=="number")return g=""+g,h!==null&&h.tag===6?(t(d,h.sibling),h=r(h,g),h.return=d,d=h):(t(d,h),h=Yu(g,d.mode,v),h.return=d,d=h),o(d);if(Na(g))return _(d,h,g,v);if(Qs(g))return y(d,h,g,v);if(R&&Da(d,g),typeof g>"u"&&!x)switch(d.tag){case 1:case 22:case 0:case 11:case 15:throw Error(j(152,Zr(d.type)||"Component"))}return t(d,h)}}var ll=F0(!0),B0=F0(!1),Xo={},Vn=Oi(Xo),zo=Oi(Xo),Vo=Oi(Xo);function sr(n){if(n===Xo)throw Error(j(174));return n}function vh(n,e){switch(dt(Vo,e),dt(zo,n),dt(Vn,Xo),n=e.nodeType,n){case 9:case 11:e=(e=e.documentElement)?e.namespaceURI:nh(null,"");break;default:n=n===8?e.parentNode:e,e=n.namespaceURI||null,n=n.tagName,e=nh(e,n)}it(Vn),dt(Vn,e)}function ls(){it(Vn),it(zo),it(Vo)}function fg(n){sr(Vo.current);var e=sr(Vn.current),t=nh(e,n.type);e!==t&&(dt(zo,n),dt(Vn,t))}function ud(n){zo.current===n&&(it(Vn),it(zo))}var ht=Oi(0);function cl(n){for(var e=n;e!==null;){if(e.tag===13){var t=e.memoizedState;if(t!==null&&(t=t.dehydrated,t===null||t.data==="$?"||t.data==="$!"))return e}else if(e.tag===19&&e.memoizedProps.revealOrder!==void 0){if(e.flags&64)return e}else if(e.child!==null){e.child.return=e,e=e.child;continue}if(e===n)break;for(;e.sibling===null;){if(e.return===null||e.return===n)return null;e=e.return}e.sibling.return=e.return,e=e.sibling}return null}var Kn=null,Si=null,Hn=!1;function k0(n,e){var t=gn(5,null,null,0);t.elementType="DELETED",t.type="DELETED",t.stateNode=e,t.return=n,t.flags=8,n.lastEffect!==null?(n.lastEffect.nextEffect=t,n.lastEffect=t):n.firstEffect=n.lastEffect=t}function pg(n,e){switch(n.tag){case 5:var t=n.type;return e=e.nodeType!==1||t.toLowerCase()!==e.nodeName.toLowerCase()?null:e,e!==null?(n.stateNode=e,!0):!1;case 6:return e=n.pendingProps===""||e.nodeType!==3?null:e,e!==null?(n.stateNode=e,!0):!1;case 13:return!1;default:return!1}}function yh(n){if(Hn){var e=Si;if(e){var t=e;if(!pg(n,e)){if(e=Qr(t.nextSibling),!e||!pg(n,e)){n.flags=n.flags&-1025|2,Hn=!1,Kn=n;return}k0(Kn,t)}Kn=n,Si=Qr(e.firstChild)}else n.flags=n.flags&-1025|2,Hn=!1,Kn=n}}function mg(n){for(n=n.return;n!==null&&n.tag!==5&&n.tag!==3&&n.tag!==13;)n=n.return;Kn=n}function Oa(n){if(n!==Kn)return!1;if(!Hn)return mg(n),Hn=!0,!1;var e=n.type;if(n.tag!==5||e!=="head"&&e!=="body"&&!fh(e,n.memoizedProps))for(e=Si;e;)k0(n,e),e=Qr(e.nextSibling);if(mg(n),n.tag===13){if(n=n.memoizedState,n=n!==null?n.dehydrated:null,!n)throw Error(j(317));e:{for(n=n.nextSibling,e=0;n;){if(n.nodeType===8){var t=n.data;if(t==="/$"){if(e===0){Si=Qr(n.nextSibling);break e}e--}else t!=="$"&&t!=="$!"&&t!=="$?"||e++}n=n.nextSibling}Si=null}}else Si=Kn?Qr(n.stateNode.nextSibling):null;return!0}function Hu(){Si=Kn=null,Hn=!1}var ts=[];function hd(){for(var n=0;n<ts.length;n++)ts[n]._workInProgressVersionPrimary=null;ts.length=0}var Eo=dr.ReactCurrentDispatcher,_n=dr.ReactCurrentBatchConfig,Ho=0,ft=null,Dt=null,Rt=null,ul=!1,Co=!1;function Zt(){throw Error(j(321))}function dd(n,e){if(e===null)return!1;for(var t=0;t<e.length&&t<n.length;t++)if(!mn(n[t],e[t]))return!1;return!0}function fd(n,e,t,i,r,s){if(Ho=s,ft=e,e.memoizedState=null,e.updateQueue=null,e.lanes=0,Eo.current=n===null||n.memoizedState===null?nw:iw,n=t(i,r),Co){s=0;do{if(Co=!1,!(25>s))throw Error(j(301));s+=1,Rt=Dt=null,e.updateQueue=null,Eo.current=rw,n=t(i,r)}while(Co)}if(Eo.current=pl,e=Dt!==null&&Dt.next!==null,Ho=0,Rt=Dt=ft=null,ul=!1,e)throw Error(j(300));return n}function or(){var n={memoizedState:null,baseState:null,baseQueue:null,queue:null,next:null};return Rt===null?ft.memoizedState=Rt=n:Rt=Rt.next=n,Rt}function pr(){if(Dt===null){var n=ft.alternate;n=n!==null?n.memoizedState:null}else n=Dt.next;var e=Rt===null?ft.memoizedState:Rt.next;if(e!==null)Rt=e,Dt=n;else{if(n===null)throw Error(j(310));Dt=n,n={memoizedState:Dt.memoizedState,baseState:Dt.baseState,baseQueue:Dt.baseQueue,queue:Dt.queue,next:null},Rt===null?ft.memoizedState=Rt=n:Rt=Rt.next=n}return Rt}function kn(n,e){return typeof e=="function"?e(n):e}function so(n){var e=pr(),t=e.queue;if(t===null)throw Error(j(311));t.lastRenderedReducer=n;var i=Dt,r=i.baseQueue,s=t.pending;if(s!==null){if(r!==null){var o=r.next;r.next=s.next,s.next=o}i.baseQueue=r=s,t.pending=null}if(r!==null){r=r.next,i=i.baseState;var a=o=s=null,l=r;do{var c=l.lane;if((Ho&c)===c)a!==null&&(a=a.next={lane:0,action:l.action,eagerReducer:l.eagerReducer,eagerState:l.eagerState,next:null}),i=l.eagerReducer===n?l.eagerState:n(i,l.action);else{var u={lane:c,action:l.action,eagerReducer:l.eagerReducer,eagerState:l.eagerState,next:null};a===null?(o=a=u,s=i):a=a.next=u,ft.lanes|=c,Yo|=c}l=l.next}while(l!==null&&l!==r);a===null?s=i:a.next=o,mn(i,e.memoizedState)||(An=!0),e.memoizedState=i,e.baseState=s,e.baseQueue=a,t.lastRenderedState=i}return[e.memoizedState,t.dispatch]}function oo(n){var e=pr(),t=e.queue;if(t===null)throw Error(j(311));t.lastRenderedReducer=n;var i=t.dispatch,r=t.pending,s=e.memoizedState;if(r!==null){t.pending=null;var o=r=r.next;do s=n(s,o.action),o=o.next;while(o!==r);mn(s,e.memoizedState)||(An=!0),e.memoizedState=s,e.baseQueue===null&&(e.baseState=s),t.lastRenderedState=s}return[s,i]}function gg(n,e,t){var i=e._getVersion;i=i(e._source);var r=e._workInProgressVersionPrimary;if(r!==null?n=r===i:(n=n.mutableReadLanes,(n=(Ho&n)===n)&&(e._workInProgressVersionPrimary=i,ts.push(e))),n)return t(e._source);throw ts.push(e),Error(j(350))}function z0(n,e,t,i){var r=Gt;if(r===null)throw Error(j(349));var s=e._getVersion,o=s(e._source),a=Eo.current,l=a.useState(function(){return gg(r,e,t)}),c=l[1],u=l[0];l=Rt;var p=n.memoizedState,f=p.refs,m=f.getSnapshot,_=p.source;p=p.subscribe;var y=ft;return n.memoizedState={refs:f,source:e,subscribe:i},a.useEffect(function(){f.getSnapshot=t,f.setSnapshot=c;var d=s(e._source);if(!mn(o,d)){d=t(e._source),mn(u,d)||(c(d),d=Ai(y),r.mutableReadLanes|=d&r.pendingLanes),d=r.mutableReadLanes,r.entangledLanes|=d;for(var h=r.entanglements,g=d;0<g;){var v=31-Li(g),x=1<<v;h[v]|=d,g&=~x}}},[t,e,i]),a.useEffect(function(){return i(e._source,function(){var d=f.getSnapshot,h=f.setSnapshot;try{h(d(e._source));var g=Ai(y);r.mutableReadLanes|=g&r.pendingLanes}catch(v){h(function(){throw v})}})},[e,i]),mn(m,t)&&mn(_,e)&&mn(p,i)||(n={pending:null,dispatch:null,lastRenderedReducer:kn,lastRenderedState:u},n.dispatch=c=gd.bind(null,ft,n),l.queue=n,l.baseQueue=null,u=gg(r,e,t),l.memoizedState=l.baseState=u),u}function V0(n,e,t){var i=pr();return z0(i,n,e,t)}function ao(n){var e=or();return typeof n=="function"&&(n=n()),e.memoizedState=e.baseState=n,n=e.queue={pending:null,dispatch:null,lastRenderedReducer:kn,lastRenderedState:n},n=n.dispatch=gd.bind(null,ft,n),[e.memoizedState,n]}function hl(n,e,t,i){return n={tag:n,create:e,destroy:t,deps:i,next:null},e=ft.updateQueue,e===null?(e={lastEffect:null},ft.updateQueue=e,e.lastEffect=n.next=n):(t=e.lastEffect,t===null?e.lastEffect=n.next=n:(i=t.next,t.next=n,n.next=i,e.lastEffect=n)),n}function _g(n){var e=or();return n={current:n},e.memoizedState=n}function dl(){return pr().memoizedState}function xh(n,e,t,i){var r=or();ft.flags|=n,r.memoizedState=hl(1|e,t,void 0,i===void 0?null:i)}function pd(n,e,t,i){var r=pr();i=i===void 0?null:i;var s=void 0;if(Dt!==null){var o=Dt.memoizedState;if(s=o.destroy,i!==null&&dd(i,o.deps)){hl(e,t,s,i);return}}ft.flags|=n,r.memoizedState=hl(1|e,t,s,i)}function vg(n,e){return xh(516,4,n,e)}function fl(n,e){return pd(516,4,n,e)}function H0(n,e){return pd(4,2,n,e)}function G0(n,e){if(typeof e=="function")return n=n(),e(n),function(){e(null)};if(e!=null)return n=n(),e.current=n,function(){e.current=null}}function W0(n,e,t){return t=t!=null?t.concat([n]):null,pd(4,2,G0.bind(null,e,n),t)}function md(){}function X0(n,e){var t=pr();e=e===void 0?null:e;var i=t.memoizedState;return i!==null&&e!==null&&dd(e,i[1])?i[0]:(t.memoizedState=[n,e],n)}function Y0(n,e){var t=pr();e=e===void 0?null:e;var i=t.memoizedState;return i!==null&&e!==null&&dd(e,i[1])?i[0]:(n=n(),t.memoizedState=[n,e],n)}function tw(n,e){var t=as();cr(98>t?98:t,function(){n(!0)}),cr(97<t?97:t,function(){var i=_n.transition;_n.transition=1;try{n(!1),e()}finally{_n.transition=i}})}function gd(n,e,t){var i=on(),r=Ai(n),s={lane:r,action:t,eagerReducer:null,eagerState:null,next:null},o=e.pending;if(o===null?s.next=s:(s.next=o.next,o.next=s),e.pending=s,o=n.alternate,n===ft||o!==null&&o===ft)Co=ul=!0;else{if(n.lanes===0&&(o===null||o.lanes===0)&&(o=e.lastRenderedReducer,o!==null))try{var a=e.lastRenderedState,l=o(a,t);if(s.eagerReducer=o,s.eagerState=l,mn(l,a))return}catch{}finally{}Ri(n,r,i)}}var pl={readContext:vn,useCallback:Zt,useContext:Zt,useEffect:Zt,useImperativeHandle:Zt,useLayoutEffect:Zt,useMemo:Zt,useReducer:Zt,useRef:Zt,useState:Zt,useDebugValue:Zt,useDeferredValue:Zt,useTransition:Zt,useMutableSource:Zt,useOpaqueIdentifier:Zt,unstable_isNewReconciler:!1},nw={readContext:vn,useCallback:function(n,e){return or().memoizedState=[n,e===void 0?null:e],n},useContext:vn,useEffect:vg,useImperativeHandle:function(n,e,t){return t=t!=null?t.concat([n]):null,xh(4,2,G0.bind(null,e,n),t)},useLayoutEffect:function(n,e){return xh(4,2,n,e)},useMemo:function(n,e){var t=or();return e=e===void 0?null:e,n=n(),t.memoizedState=[n,e],n},useReducer:function(n,e,t){var i=or();return e=t!==void 0?t(e):e,i.memoizedState=i.baseState=e,n=i.queue={pending:null,dispatch:null,lastRenderedReducer:n,lastRenderedState:e},n=n.dispatch=gd.bind(null,ft,n),[i.memoizedState,n]},useRef:_g,useState:ao,useDebugValue:md,useDeferredValue:function(n){var e=ao(n),t=e[0],i=e[1];return vg(function(){var r=_n.transition;_n.transition=1;try{i(n)}finally{_n.transition=r}},[n]),t},useTransition:function(){var n=ao(!1),e=n[0];return n=tw.bind(null,n[1]),_g(n),[n,e]},useMutableSource:function(n,e,t){var i=or();return i.memoizedState={refs:{getSnapshot:e,setSnapshot:null},source:n,subscribe:t},z0(i,n,e,t)},useOpaqueIdentifier:function(){if(Hn){var n=!1,e=Zx(function(){throw n||(n=!0,t("r:"+(ku++).toString(36))),Error(j(355))}),t=ao(e)[1];return!(ft.mode&2)&&(ft.flags|=516,hl(5,function(){t("r:"+(ku++).toString(36))},void 0,null)),e}return e="r:"+(ku++).toString(36),ao(e),e},unstable_isNewReconciler:!1},iw={readContext:vn,useCallback:X0,useContext:vn,useEffect:fl,useImperativeHandle:W0,useLayoutEffect:H0,useMemo:Y0,useReducer:so,useRef:dl,useState:function(){return so(kn)},useDebugValue:md,useDeferredValue:function(n){var e=so(kn),t=e[0],i=e[1];return fl(function(){var r=_n.transition;_n.transition=1;try{i(n)}finally{_n.transition=r}},[n]),t},useTransition:function(){var n=so(kn)[0];return[dl().current,n]},useMutableSource:V0,useOpaqueIdentifier:function(){return so(kn)[0]},unstable_isNewReconciler:!1},rw={readContext:vn,useCallback:X0,useContext:vn,useEffect:fl,useImperativeHandle:W0,useLayoutEffect:H0,useMemo:Y0,useReducer:oo,useRef:dl,useState:function(){return oo(kn)},useDebugValue:md,useDeferredValue:function(n){var e=oo(kn),t=e[0],i=e[1];return fl(function(){var r=_n.transition;_n.transition=1;try{i(n)}finally{_n.transition=r}},[n]),t},useTransition:function(){var n=oo(kn)[0];return[dl().current,n]},useMutableSource:V0,useOpaqueIdentifier:function(){return oo(kn)[0]},unstable_isNewReconciler:!1},sw=dr.ReactCurrentOwner,An=!1;function Jt(n,e,t,i){e.child=n===null?B0(e,null,t,i):ll(e,n.child,t,i)}function yg(n,e,t,i,r){t=t.render;var s=e.ref;return es(e,r),i=fd(n,e,t,i,s,r),n!==null&&!An?(e.updateQueue=n.updateQueue,e.flags&=-517,n.lanes&=~r,jn(n,e,r)):(e.flags|=1,Jt(n,e,i,r),e.child)}function xg(n,e,t,i,r,s){if(n===null){var o=t.type;return typeof o=="function"&&!Sd(o)&&o.defaultProps===void 0&&t.compare===null&&t.defaultProps===void 0?(e.tag=15,e.type=o,q0(n,e,o,i,r,s)):(n=$a(t.type,null,i,e,e.mode,s),n.ref=e.ref,n.return=e,e.child=n)}return o=n.child,!(r&s)&&(r=o.memoizedProps,t=t.compare,t=t!==null?t:Oo,t(r,i)&&n.ref===e.ref)?jn(n,e,s):(e.flags|=1,n=Di(o,i),n.ref=e.ref,n.return=e,e.child=n)}function q0(n,e,t,i,r,s){if(n!==null&&Oo(n.memoizedProps,i)&&n.ref===e.ref)if(An=!1,(s&r)!==0)n.flags&16384&&(An=!0);else return e.lanes=n.lanes,jn(n,e,s);return wh(n,e,t,i,s)}function Gu(n,e,t){var i=e.pendingProps,r=i.children,s=n!==null?n.memoizedState:null;if(i.mode==="hidden"||i.mode==="unstable-defer-without-hiding")if(!(e.mode&4))e.memoizedState={baseLanes:0},Ba(e,t);else if(t&1073741824)e.memoizedState={baseLanes:0},Ba(e,s!==null?s.baseLanes:t);else return n=s!==null?s.baseLanes|t:t,e.lanes=e.childLanes=1073741824,e.memoizedState={baseLanes:n},Ba(e,n),null;else s!==null?(i=s.baseLanes|t,e.memoizedState=null):i=t,Ba(e,i);return Jt(n,e,r,t),e.child}function $0(n,e){var t=e.ref;(n===null&&t!==null||n!==null&&n.ref!==t)&&(e.flags|=128)}function wh(n,e,t,i,r){var s=jt(t)?lr:Bt.current;return s=os(e,s),es(e,r),t=fd(n,e,t,i,s,r),n!==null&&!An?(e.updateQueue=n.updateQueue,e.flags&=-517,n.lanes&=~r,jn(n,e,r)):(e.flags|=1,Jt(n,e,t,r),e.child)}function wg(n,e,t,i,r){if(jt(t)){var s=!0;Ga(e)}else s=!1;if(es(e,r),e.stateNode===null)n!==null&&(n.alternate=null,e.alternate=null,e.flags|=2),O0(e,t,i),_h(e,t,i,r),i=!0;else if(n===null){var o=e.stateNode,a=e.memoizedProps;o.props=a;var l=o.context,c=t.contextType;typeof c=="object"&&c!==null?c=vn(c):(c=jt(t)?lr:Bt.current,c=os(e,c));var u=t.getDerivedStateFromProps,p=typeof u=="function"||typeof o.getSnapshotBeforeUpdate=="function";p||typeof o.UNSAFE_componentWillReceiveProps!="function"&&typeof o.componentWillReceiveProps!="function"||(a!==i||l!==c)&&dg(e,o,i,c),vi=!1;var f=e.memoizedState;o.state=f,ko(e,i,o,r),l=e.memoizedState,a!==i||f!==l||Kt.current||vi?(typeof u=="function"&&(al(e,t,u,i),l=e.memoizedState),(a=vi||hg(e,t,a,i,f,l,c))?(p||typeof o.UNSAFE_componentWillMount!="function"&&typeof o.componentWillMount!="function"||(typeof o.componentWillMount=="function"&&o.componentWillMount(),typeof o.UNSAFE_componentWillMount=="function"&&o.UNSAFE_componentWillMount()),typeof o.componentDidMount=="function"&&(e.flags|=4)):(typeof o.componentDidMount=="function"&&(e.flags|=4),e.memoizedProps=i,e.memoizedState=l),o.props=i,o.state=l,o.context=c,i=a):(typeof o.componentDidMount=="function"&&(e.flags|=4),i=!1)}else{o=e.stateNode,N0(n,e),a=e.memoizedProps,c=e.type===e.elementType?a:bn(e.type,a),o.props=c,p=e.pendingProps,f=o.context,l=t.contextType,typeof l=="object"&&l!==null?l=vn(l):(l=jt(t)?lr:Bt.current,l=os(e,l));var m=t.getDerivedStateFromProps;(u=typeof m=="function"||typeof o.getSnapshotBeforeUpdate=="function")||typeof o.UNSAFE_componentWillReceiveProps!="function"&&typeof o.componentWillReceiveProps!="function"||(a!==p||f!==l)&&dg(e,o,i,l),vi=!1,f=e.memoizedState,o.state=f,ko(e,i,o,r);var _=e.memoizedState;a!==p||f!==_||Kt.current||vi?(typeof m=="function"&&(al(e,t,m,i),_=e.memoizedState),(c=vi||hg(e,t,c,i,f,_,l))?(u||typeof o.UNSAFE_componentWillUpdate!="function"&&typeof o.componentWillUpdate!="function"||(typeof o.componentWillUpdate=="function"&&o.componentWillUpdate(i,_,l),typeof o.UNSAFE_componentWillUpdate=="function"&&o.UNSAFE_componentWillUpdate(i,_,l)),typeof o.componentDidUpdate=="function"&&(e.flags|=4),typeof o.getSnapshotBeforeUpdate=="function"&&(e.flags|=256)):(typeof o.componentDidUpdate!="function"||a===n.memoizedProps&&f===n.memoizedState||(e.flags|=4),typeof o.getSnapshotBeforeUpdate!="function"||a===n.memoizedProps&&f===n.memoizedState||(e.flags|=256),e.memoizedProps=i,e.memoizedState=_),o.props=i,o.state=_,o.context=l,i=c):(typeof o.componentDidUpdate!="function"||a===n.memoizedProps&&f===n.memoizedState||(e.flags|=4),typeof o.getSnapshotBeforeUpdate!="function"||a===n.memoizedProps&&f===n.memoizedState||(e.flags|=256),i=!1)}return Sh(n,e,t,i,s,r)}function Sh(n,e,t,i,r,s){$0(n,e);var o=(e.flags&64)!==0;if(!i&&!o)return r&&og(e,t,!1),jn(n,e,s);i=e.stateNode,sw.current=e;var a=o&&typeof t.getDerivedStateFromError!="function"?null:i.render();return e.flags|=1,n!==null&&o?(e.child=ll(e,n.child,null,s),e.child=ll(e,null,a,s)):Jt(n,e,a,s),e.memoizedState=i.state,r&&og(e,t,!0),e.child}function Sg(n){var e=n.stateNode;e.pendingContext?sg(n,e.pendingContext,e.pendingContext!==e.context):e.context&&sg(n,e.context,!1),vh(n,e.containerInfo)}var Fa={dehydrated:null,retryLane:0};function Mg(n,e,t){var i=e.pendingProps,r=ht.current,s=!1,o;return(o=(e.flags&64)!==0)||(o=n!==null&&n.memoizedState===null?!1:(r&2)!==0),o?(s=!0,e.flags&=-65):n!==null&&n.memoizedState===null||i.fallback===void 0||i.unstable_avoidThisFallback===!0||(r|=1),dt(ht,r&1),n===null?(i.fallback!==void 0&&yh(e),n=i.children,r=i.fallback,s?(n=Eg(e,n,r,t),e.child.memoizedState={baseLanes:t},e.memoizedState=Fa,n):typeof i.unstable_expectedLoadTime=="number"?(n=Eg(e,n,r,t),e.child.memoizedState={baseLanes:t},e.memoizedState=Fa,e.lanes=33554432,n):(t=Md({mode:"visible",children:n},e.mode,t,null),t.return=e,e.child=t)):n.memoizedState!==null?s?(i=Tg(n,e,i.children,i.fallback,t),s=e.child,r=n.child.memoizedState,s.memoizedState=r===null?{baseLanes:t}:{baseLanes:r.baseLanes|t},s.childLanes=n.childLanes&~t,e.memoizedState=Fa,i):(t=Cg(n,e,i.children,t),e.memoizedState=null,t):s?(i=Tg(n,e,i.children,i.fallback,t),s=e.child,r=n.child.memoizedState,s.memoizedState=r===null?{baseLanes:t}:{baseLanes:r.baseLanes|t},s.childLanes=n.childLanes&~t,e.memoizedState=Fa,i):(t=Cg(n,e,i.children,t),e.memoizedState=null,t)}function Eg(n,e,t,i){var r=n.mode,s=n.child;return e={mode:"hidden",children:e},!(r&2)&&s!==null?(s.childLanes=0,s.pendingProps=e):s=Md(e,r,0,null),t=rs(t,r,i,null),s.return=n,t.return=n,s.sibling=t,n.child=s,t}function Cg(n,e,t,i){var r=n.child;return n=r.sibling,t=Di(r,{mode:"visible",children:t}),!(e.mode&2)&&(t.lanes=i),t.return=e,t.sibling=null,n!==null&&(n.nextEffect=null,n.flags=8,e.firstEffect=e.lastEffect=n),e.child=t}function Tg(n,e,t,i,r){var s=e.mode,o=n.child;n=o.sibling;var a={mode:"hidden",children:t};return!(s&2)&&e.child!==o?(t=e.child,t.childLanes=0,t.pendingProps=a,o=t.lastEffect,o!==null?(e.firstEffect=t.firstEffect,e.lastEffect=o,o.nextEffect=null):e.firstEffect=e.lastEffect=null):t=Di(o,a),n!==null?i=Di(n,i):(i=rs(i,s,r,null),i.flags|=2),i.return=e,t.return=e,t.sibling=i,e.child=t,i}function bg(n,e){n.lanes|=e;var t=n.alternate;t!==null&&(t.lanes|=e),U0(n.return,e)}function Wu(n,e,t,i,r,s){var o=n.memoizedState;o===null?n.memoizedState={isBackwards:e,rendering:null,renderingStartTime:0,last:i,tail:t,tailMode:r,lastEffect:s}:(o.isBackwards=e,o.rendering=null,o.renderingStartTime=0,o.last=i,o.tail=t,o.tailMode=r,o.lastEffect=s)}function Ag(n,e,t){var i=e.pendingProps,r=i.revealOrder,s=i.tail;if(Jt(n,e,i.children,t),i=ht.current,i&2)i=i&1|2,e.flags|=64;else{if(n!==null&&n.flags&64)e:for(n=e.child;n!==null;){if(n.tag===13)n.memoizedState!==null&&bg(n,t);else if(n.tag===19)bg(n,t);else if(n.child!==null){n.child.return=n,n=n.child;continue}if(n===e)break e;for(;n.sibling===null;){if(n.return===null||n.return===e)break e;n=n.return}n.sibling.return=n.return,n=n.sibling}i&=1}if(dt(ht,i),!(e.mode&2))e.memoizedState=null;else switch(r){case"forwards":for(t=e.child,r=null;t!==null;)n=t.alternate,n!==null&&cl(n)===null&&(r=t),t=t.sibling;t=r,t===null?(r=e.child,e.child=null):(r=t.sibling,t.sibling=null),Wu(e,!1,r,t,s,e.lastEffect);break;case"backwards":for(t=null,r=e.child,e.child=null;r!==null;){if(n=r.alternate,n!==null&&cl(n)===null){e.child=r;break}n=r.sibling,r.sibling=t,t=r,r=n}Wu(e,!0,t,null,s,e.lastEffect);break;case"together":Wu(e,!1,null,null,void 0,e.lastEffect);break;default:e.memoizedState=null}return e.child}function jn(n,e,t){if(n!==null&&(e.dependencies=n.dependencies),Yo|=e.lanes,t&e.childLanes){if(n!==null&&e.child!==n.child)throw Error(j(153));if(e.child!==null){for(n=e.child,t=Di(n,n.pendingProps),e.child=t,t.return=e;n.sibling!==null;)n=n.sibling,t=t.sibling=Di(n,n.pendingProps),t.return=e;t.sibling=null}return e.child}return null}var Z0,Mh,J0,K0;Z0=function(n,e){for(var t=e.child;t!==null;){if(t.tag===5||t.tag===6)n.appendChild(t.stateNode);else if(t.tag!==4&&t.child!==null){t.child.return=t,t=t.child;continue}if(t===e)break;for(;t.sibling===null;){if(t.return===null||t.return===e)return;t=t.return}t.sibling.return=t.return,t=t.sibling}};Mh=function(){};J0=function(n,e,t,i){var r=n.memoizedProps;if(r!==i){n=e.stateNode,sr(Vn.current);var s=null;switch(t){case"input":r=Ju(n,r),i=Ju(n,i),s=[];break;case"option":r=Qu(n,r),i=Qu(n,i),s=[];break;case"select":r=st({},r,{value:void 0}),i=st({},i,{value:void 0}),s=[];break;case"textarea":r=eh(n,r),i=eh(n,i),s=[];break;default:typeof r.onClick!="function"&&typeof i.onClick=="function"&&(n.onclick=tl)}ih(t,i);var o;t=null;for(c in r)if(!i.hasOwnProperty(c)&&r.hasOwnProperty(c)&&r[c]!=null)if(c==="style"){var a=r[c];for(o in a)a.hasOwnProperty(o)&&(t||(t={}),t[o]="")}else c!=="dangerouslySetInnerHTML"&&c!=="children"&&c!=="suppressContentEditableWarning"&&c!=="suppressHydrationWarning"&&c!=="autoFocus"&&(Ro.hasOwnProperty(c)?s||(s=[]):(s=s||[]).push(c,null));for(c in i){var l=i[c];if(a=r?.[c],i.hasOwnProperty(c)&&l!==a&&(l!=null||a!=null))if(c==="style")if(a){for(o in a)!a.hasOwnProperty(o)||l&&l.hasOwnProperty(o)||(t||(t={}),t[o]="");for(o in l)l.hasOwnProperty(o)&&a[o]!==l[o]&&(t||(t={}),t[o]=l[o])}else t||(s||(s=[]),s.push(c,t)),t=l;else c==="dangerouslySetInnerHTML"?(l=l?l.__html:void 0,a=a?a.__html:void 0,l!=null&&a!==l&&(s=s||[]).push(c,l)):c==="children"?typeof l!="string"&&typeof l!="number"||(s=s||[]).push(c,""+l):c!=="suppressContentEditableWarning"&&c!=="suppressHydrationWarning"&&(Ro.hasOwnProperty(c)?(l!=null&&c==="onScroll"&&nt("scroll",n),s||a===l||(s=[])):typeof l=="object"&&l!==null&&l.$$typeof===Gh?l.toString():(s=s||[]).push(c,l))}t&&(s=s||[]).push("style",t);var c=s;(e.updateQueue=c)&&(e.flags|=4)}};K0=function(n,e,t,i){t!==i&&(e.flags|=4)};function lo(n,e){if(!Hn)switch(n.tailMode){case"hidden":e=n.tail;for(var t=null;e!==null;)e.alternate!==null&&(t=e),e=e.sibling;t===null?n.tail=null:t.sibling=null;break;case"collapsed":t=n.tail;for(var i=null;t!==null;)t.alternate!==null&&(i=t),t=t.sibling;i===null?e||n.tail===null?n.tail=null:n.tail.sibling=null:i.sibling=null}}function ow(n,e,t){var i=e.pendingProps;switch(e.tag){case 2:case 16:case 15:case 0:case 11:case 7:case 8:case 12:case 9:case 14:return null;case 1:return jt(e.type)&&il(),null;case 3:return ls(),it(Kt),it(Bt),hd(),i=e.stateNode,i.pendingContext&&(i.context=i.pendingContext,i.pendingContext=null),(n===null||n.child===null)&&(Oa(e)?e.flags|=4:i.hydrate||(e.flags|=256)),Mh(e),null;case 5:ud(e);var r=sr(Vo.current);if(t=e.type,n!==null&&e.stateNode!=null)J0(n,e,t,i,r),n.ref!==e.ref&&(e.flags|=128);else{if(!i){if(e.stateNode===null)throw Error(j(166));return null}if(n=sr(Vn.current),Oa(e)){i=e.stateNode,t=e.type;var s=e.memoizedProps;switch(i[wi]=e,i[nl]=s,t){case"dialog":nt("cancel",i),nt("close",i);break;case"iframe":case"object":case"embed":nt("load",i);break;case"video":case"audio":for(n=0;n<po.length;n++)nt(po[n],i);break;case"source":nt("error",i);break;case"img":case"image":case"link":nt("error",i),nt("load",i);break;case"details":nt("toggle",i);break;case"input":Pm(i,s),nt("invalid",i);break;case"select":i._wrapperState={wasMultiple:!!s.multiple},nt("invalid",i);break;case"textarea":Lm(i,s),nt("invalid",i)}ih(t,s),n=null;for(var o in s)s.hasOwnProperty(o)&&(r=s[o],o==="children"?typeof r=="string"?i.textContent!==r&&(n=["children",r]):typeof r=="number"&&i.textContent!==""+r&&(n=["children",""+r]):Ro.hasOwnProperty(o)&&r!=null&&o==="onScroll"&&nt("scroll",i));switch(t){case"input":ba(i),Im(i,s,!0);break;case"textarea":ba(i),Um(i);break;case"select":case"option":break;default:typeof s.onClick=="function"&&(i.onclick=tl)}i=n,e.updateQueue=i,i!==null&&(e.flags|=4)}else{switch(o=r.nodeType===9?r:r.ownerDocument,n===th.html&&(n=Wg(t)),n===th.html?t==="script"?(n=o.createElement("div"),n.innerHTML="<script><\/script>",n=n.removeChild(n.firstChild)):typeof i.is=="string"?n=o.createElement(t,{is:i.is}):(n=o.createElement(t),t==="select"&&(o=n,i.multiple?o.multiple=!0:i.size&&(o.size=i.size))):n=o.createElementNS(n,t),n[wi]=e,n[nl]=i,Z0(n,e,!1,!1),e.stateNode=n,o=rh(t,i),t){case"dialog":nt("cancel",n),nt("close",n),r=i;break;case"iframe":case"object":case"embed":nt("load",n),r=i;break;case"video":case"audio":for(r=0;r<po.length;r++)nt(po[r],n);r=i;break;case"source":nt("error",n),r=i;break;case"img":case"image":case"link":nt("error",n),nt("load",n),r=i;break;case"details":nt("toggle",n),r=i;break;case"input":Pm(n,i),r=Ju(n,i),nt("invalid",n);break;case"option":r=Qu(n,i);break;case"select":n._wrapperState={wasMultiple:!!i.multiple},r=st({},i,{value:void 0}),nt("invalid",n);break;case"textarea":Lm(n,i),r=eh(n,i),nt("invalid",n);break;default:r=i}ih(t,r);var a=r;for(s in a)if(a.hasOwnProperty(s)){var l=a[s];s==="style"?qg(n,l):s==="dangerouslySetInnerHTML"?(l=l?l.__html:void 0,l!=null&&Xg(n,l)):s==="children"?typeof l=="string"?(t!=="textarea"||l!=="")&&Po(n,l):typeof l=="number"&&Po(n,""+l):s!=="suppressContentEditableWarning"&&s!=="suppressHydrationWarning"&&s!=="autoFocus"&&(Ro.hasOwnProperty(s)?l!=null&&s==="onScroll"&&nt("scroll",n):l!=null&&Fh(n,s,l,o))}switch(t){case"input":ba(n),Im(n,i,!1);break;case"textarea":ba(n),Um(n);break;case"option":i.value!=null&&n.setAttribute("value",""+Ii(i.value));break;case"select":n.multiple=!!i.multiple,s=i.value,s!=null?Jr(n,!!i.multiple,s,!1):i.defaultValue!=null&&Jr(n,!!i.multiple,i.defaultValue,!0);break;default:typeof r.onClick=="function"&&(n.onclick=tl)}E0(t,i)&&(e.flags|=4)}e.ref!==null&&(e.flags|=128)}return null;case 6:if(n&&e.stateNode!=null)K0(n,e,n.memoizedProps,i);else{if(typeof i!="string"&&e.stateNode===null)throw Error(j(166));t=sr(Vo.current),sr(Vn.current),Oa(e)?(i=e.stateNode,t=e.memoizedProps,i[wi]=e,i.nodeValue!==t&&(e.flags|=4)):(i=(t.nodeType===9?t:t.ownerDocument).createTextNode(i),i[wi]=e,e.stateNode=i)}return null;case 13:return it(ht),i=e.memoizedState,e.flags&64?(e.lanes=t,e):(i=i!==null,t=!1,n===null?e.memoizedProps.fallback!==void 0&&Oa(e):t=n.memoizedState!==null,i&&!t&&e.mode&2&&(n===null&&e.memoizedProps.unstable_avoidThisFallback!==!0||ht.current&1?Pt===0&&(Pt=3):((Pt===0||Pt===3)&&(Pt=4),Gt===null||!(Yo&134217727)&&!(ds&134217727)||ns(Gt,Ft))),(i||t)&&(e.flags|=4),null);case 4:return ls(),Mh(e),n===null&&w0(e.stateNode.containerInfo),null;case 10:return ld(e),null;case 17:return jt(e.type)&&il(),null;case 19:if(it(ht),i=e.memoizedState,i===null)return null;if(s=(e.flags&64)!==0,o=i.rendering,o===null)if(s)lo(i,!1);else{if(Pt!==0||n!==null&&n.flags&64)for(n=e.child;n!==null;){if(o=cl(n),o!==null){for(e.flags|=64,lo(i,!1),s=o.updateQueue,s!==null&&(e.updateQueue=s,e.flags|=4),i.lastEffect===null&&(e.firstEffect=null),e.lastEffect=i.lastEffect,i=t,t=e.child;t!==null;)s=t,n=i,s.flags&=2,s.nextEffect=null,s.firstEffect=null,s.lastEffect=null,o=s.alternate,o===null?(s.childLanes=0,s.lanes=n,s.child=null,s.memoizedProps=null,s.memoizedState=null,s.updateQueue=null,s.dependencies=null,s.stateNode=null):(s.childLanes=o.childLanes,s.lanes=o.lanes,s.child=o.child,s.memoizedProps=o.memoizedProps,s.memoizedState=o.memoizedState,s.updateQueue=o.updateQueue,s.type=o.type,n=o.dependencies,s.dependencies=n===null?null:{lanes:n.lanes,firstContext:n.firstContext}),t=t.sibling;return dt(ht,ht.current&1|2),e.child}n=n.sibling}i.tail!==null&&Ot()>Rh&&(e.flags|=64,s=!0,lo(i,!1),e.lanes=33554432)}else{if(!s)if(n=cl(o),n!==null){if(e.flags|=64,s=!0,t=n.updateQueue,t!==null&&(e.updateQueue=t,e.flags|=4),lo(i,!0),i.tail===null&&i.tailMode==="hidden"&&!o.alternate&&!Hn)return e=e.lastEffect=i.lastEffect,e!==null&&(e.nextEffect=null),null}else 2*Ot()-i.renderingStartTime>Rh&&t!==1073741824&&(e.flags|=64,s=!0,lo(i,!1),e.lanes=33554432);i.isBackwards?(o.sibling=e.child,e.child=o):(t=i.last,t!==null?t.sibling=o:e.child=o,i.last=o)}return i.tail!==null?(t=i.tail,i.rendering=t,i.tail=t.sibling,i.lastEffect=e.lastEffect,i.renderingStartTime=Ot(),t.sibling=null,e=ht.current,dt(ht,s?e&1|2:e&1),t):null;case 23:case 24:return wd(),n!==null&&n.memoizedState!==null!=(e.memoizedState!==null)&&i.mode!=="unstable-defer-without-hiding"&&(e.flags|=4),null}throw Error(j(156,e.tag))}function aw(n){switch(n.tag){case 1:jt(n.type)&&il();var e=n.flags;return e&4096?(n.flags=e&-4097|64,n):null;case 3:if(ls(),it(Kt),it(Bt),hd(),e=n.flags,e&64)throw Error(j(285));return n.flags=e&-4097|64,n;case 5:return ud(n),null;case 13:return it(ht),e=n.flags,e&4096?(n.flags=e&-4097|64,n):null;case 19:return it(ht),null;case 4:return ls(),null;case 10:return ld(n),null;case 23:case 24:return wd(),null;default:return null}}function _d(n,e){try{var t="",i=e;do t+=Vy(i),i=i.return;while(i);var r=t}catch(s){r=`
Error generating stack: `+s.message+`
`+s.stack}return{value:n,source:e,stack:r}}function Eh(n,e){try{console.error(e.value)}catch(t){setTimeout(function(){throw t})}}var lw=typeof WeakMap=="function"?WeakMap:Map;function j0(n,e,t){t=Ti(-1,t),t.tag=3,t.payload={element:null};var i=e.value;return t.callback=function(){gl||(gl=!0,Ph=i),Eh(n,e)},t}function Q0(n,e,t){t=Ti(-1,t),t.tag=3;var i=n.type.getDerivedStateFromError;if(typeof i=="function"){var r=e.value;t.payload=function(){return Eh(n,e),i(r)}}var s=n.stateNode;return s!==null&&typeof s.componentDidCatch=="function"&&(t.callback=function(){typeof i!="function"&&(zn===null?zn=new Set([this]):zn.add(this),Eh(n,e));var o=e.stack;this.componentDidCatch(e.value,{componentStack:o!==null?o:""})}),t}var cw=typeof WeakSet=="function"?WeakSet:Set;function Rg(n){var e=n.ref;if(e!==null)if(typeof e=="function")try{e(null)}catch(t){Pi(n,t)}else e.current=null}function uw(n,e){switch(e.tag){case 0:case 11:case 15:case 22:return;case 1:if(e.flags&256&&n!==null){var t=n.memoizedProps,i=n.memoizedState;n=e.stateNode,e=n.getSnapshotBeforeUpdate(e.elementType===e.type?t:bn(e.type,t),i),n.__reactInternalSnapshotBeforeUpdate=e}return;case 3:e.flags&256&&rd(e.stateNode.containerInfo);return;case 5:case 6:case 4:case 17:return}throw Error(j(163))}function hw(n,e,t){switch(t.tag){case 0:case 11:case 15:case 22:if(e=t.updateQueue,e=e!==null?e.lastEffect:null,e!==null){n=e=e.next;do{if((n.tag&3)===3){var i=n.create;n.destroy=i()}n=n.next}while(n!==e)}if(e=t.updateQueue,e=e!==null?e.lastEffect:null,e!==null){n=e=e.next;do{var r=n;i=r.next,r=r.tag,r&4&&r&1&&(l_(t,n),yw(t,n)),n=i}while(n!==e)}return;case 1:n=t.stateNode,t.flags&4&&(e===null?n.componentDidMount():(i=t.elementType===t.type?e.memoizedProps:bn(t.type,e.memoizedProps),n.componentDidUpdate(i,e.memoizedState,n.__reactInternalSnapshotBeforeUpdate))),e=t.updateQueue,e!==null&&ug(t,e,n);return;case 3:if(e=t.updateQueue,e!==null){if(n=null,t.child!==null)switch(t.child.tag){case 5:n=t.child.stateNode;break;case 1:n=t.child.stateNode}ug(t,e,n)}return;case 5:n=t.stateNode,e===null&&t.flags&4&&E0(t.type,t.memoizedProps)&&n.focus();return;case 6:return;case 4:return;case 12:return;case 13:t.memoizedState===null&&(t=t.alternate,t!==null&&(t=t.memoizedState,t!==null&&(t=t.dehydrated,t!==null&&i0(t))));return;case 19:case 17:case 20:case 21:case 23:case 24:return}throw Error(j(163))}function Pg(n,e){for(var t=n;;){if(t.tag===5){var i=t.stateNode;if(e)i=i.style,typeof i.setProperty=="function"?i.setProperty("display","none","important"):i.display="none";else{i=t.stateNode;var r=t.memoizedProps.style;r=r!=null&&r.hasOwnProperty("display")?r.display:null,i.style.display=Yg("display",r)}}else if(t.tag===6)t.stateNode.nodeValue=e?"":t.memoizedProps;else if((t.tag!==23&&t.tag!==24||t.memoizedState===null||t===n)&&t.child!==null){t.child.return=t,t=t.child;continue}if(t===n)break;for(;t.sibling===null;){if(t.return===null||t.return===n)return;t=t.return}t.sibling.return=t.return,t=t.sibling}}function Ig(n,e){if(ar&&typeof ar.onCommitFiberUnmount=="function")try{ar.onCommitFiberUnmount(sd,e)}catch{}switch(e.tag){case 0:case 11:case 14:case 15:case 22:if(n=e.updateQueue,n!==null&&(n=n.lastEffect,n!==null)){var t=n=n.next;do{var i=t,r=i.destroy;if(i=i.tag,r!==void 0)if(i&4)l_(e,t);else{i=e;try{r()}catch(s){Pi(i,s)}}t=t.next}while(t!==n)}break;case 1:if(Rg(e),n=e.stateNode,typeof n.componentWillUnmount=="function")try{n.props=e.memoizedProps,n.state=e.memoizedState,n.componentWillUnmount()}catch(s){Pi(e,s)}break;case 5:Rg(e);break;case 4:e_(n,e)}}function Lg(n){n.alternate=null,n.child=null,n.dependencies=null,n.firstEffect=null,n.lastEffect=null,n.memoizedProps=null,n.memoizedState=null,n.pendingProps=null,n.return=null,n.updateQueue=null}function Ug(n){return n.tag===5||n.tag===3||n.tag===4}function Ng(n){e:{for(var e=n.return;e!==null;){if(Ug(e))break e;e=e.return}throw Error(j(160))}var t=e;switch(e=t.stateNode,t.tag){case 5:var i=!1;break;case 3:e=e.containerInfo,i=!0;break;case 4:e=e.containerInfo,i=!0;break;default:throw Error(j(161))}t.flags&16&&(Po(e,""),t.flags&=-17);e:t:for(t=n;;){for(;t.sibling===null;){if(t.return===null||Ug(t.return)){t=null;break e}t=t.return}for(t.sibling.return=t.return,t=t.sibling;t.tag!==5&&t.tag!==6&&t.tag!==18;){if(t.flags&2||t.child===null||t.tag===4)continue t;t.child.return=t,t=t.child}if(!(t.flags&2)){t=t.stateNode;break e}}i?Ch(n,t,e):Th(n,t,e)}function Ch(n,e,t){var i=n.tag,r=i===5||i===6;if(r)n=r?n.stateNode:n.stateNode.instance,e?t.nodeType===8?t.parentNode.insertBefore(n,e):t.insertBefore(n,e):(t.nodeType===8?(e=t.parentNode,e.insertBefore(n,t)):(e=t,e.appendChild(n)),t=t._reactRootContainer,t!=null||e.onclick!==null||(e.onclick=tl));else if(i!==4&&(n=n.child,n!==null))for(Ch(n,e,t),n=n.sibling;n!==null;)Ch(n,e,t),n=n.sibling}function Th(n,e,t){var i=n.tag,r=i===5||i===6;if(r)n=r?n.stateNode:n.stateNode.instance,e?t.insertBefore(n,e):t.appendChild(n);else if(i!==4&&(n=n.child,n!==null))for(Th(n,e,t),n=n.sibling;n!==null;)Th(n,e,t),n=n.sibling}function e_(n,e){for(var t=e,i=!1,r,s;;){if(!i){i=t.return;e:for(;;){if(i===null)throw Error(j(160));switch(r=i.stateNode,i.tag){case 5:s=!1;break e;case 3:r=r.containerInfo,s=!0;break e;case 4:r=r.containerInfo,s=!0;break e}i=i.return}i=!0}if(t.tag===5||t.tag===6){e:for(var o=n,a=t,l=a;;)if(Ig(o,l),l.child!==null&&l.tag!==4)l.child.return=l,l=l.child;else{if(l===a)break e;for(;l.sibling===null;){if(l.return===null||l.return===a)break e;l=l.return}l.sibling.return=l.return,l=l.sibling}s?(o=r,a=t.stateNode,o.nodeType===8?o.parentNode.removeChild(a):o.removeChild(a)):r.removeChild(t.stateNode)}else if(t.tag===4){if(t.child!==null){r=t.stateNode.containerInfo,s=!0,t.child.return=t,t=t.child;continue}}else if(Ig(n,t),t.child!==null){t.child.return=t,t=t.child;continue}if(t===e)break;for(;t.sibling===null;){if(t.return===null||t.return===e)return;t=t.return,t.tag===4&&(i=!1)}t.sibling.return=t.return,t=t.sibling}}function Xu(n,e){switch(e.tag){case 0:case 11:case 14:case 15:case 22:var t=e.updateQueue;if(t=t!==null?t.lastEffect:null,t!==null){var i=t=t.next;do(i.tag&3)===3&&(n=i.destroy,i.destroy=void 0,n!==void 0&&n()),i=i.next;while(i!==t)}return;case 1:return;case 5:if(t=e.stateNode,t!=null){i=e.memoizedProps;var r=n!==null?n.memoizedProps:i;n=e.type;var s=e.updateQueue;if(e.updateQueue=null,s!==null){for(t[nl]=i,n==="input"&&i.type==="radio"&&i.name!=null&&Hg(t,i),rh(n,r),e=rh(n,i),r=0;r<s.length;r+=2){var o=s[r],a=s[r+1];o==="style"?qg(t,a):o==="dangerouslySetInnerHTML"?Xg(t,a):o==="children"?Po(t,a):Fh(t,o,a,e)}switch(n){case"input":Ku(t,i);break;case"textarea":Gg(t,i);break;case"select":n=t._wrapperState.wasMultiple,t._wrapperState.wasMultiple=!!i.multiple,s=i.value,s!=null?Jr(t,!!i.multiple,s,!1):n!==!!i.multiple&&(i.defaultValue!=null?Jr(t,!!i.multiple,i.defaultValue,!0):Jr(t,!!i.multiple,i.multiple?[]:"",!1))}}}return;case 6:if(e.stateNode===null)throw Error(j(162));e.stateNode.nodeValue=e.memoizedProps;return;case 3:t=e.stateNode,t.hydrate&&(t.hydrate=!1,i0(t.containerInfo));return;case 12:return;case 13:e.memoizedState!==null&&(xd=Ot(),Pg(e.child,!0)),Dg(e);return;case 19:Dg(e);return;case 17:return;case 23:case 24:Pg(e,e.memoizedState!==null);return}throw Error(j(163))}function Dg(n){var e=n.updateQueue;if(e!==null){n.updateQueue=null;var t=n.stateNode;t===null&&(t=n.stateNode=new cw),e.forEach(function(i){var r=Sw.bind(null,n,i);t.has(i)||(t.add(i),i.then(r,r))})}}function dw(n,e){return n!==null&&(n=n.memoizedState,n===null||n.dehydrated!==null)?(e=e.memoizedState,e!==null&&e.dehydrated===null):!1}var fw=Math.ceil,ml=dr.ReactCurrentDispatcher,vd=dr.ReactCurrentOwner,Ce=0,Gt=null,xt=null,Ft=0,ur=0,bh=Oi(0),Pt=0,Rl=null,hs=0,Yo=0,ds=0,yd=0,Ah=null,xd=0,Rh=1/0;function fs(){Rh=Ot()+500}var le=null,gl=!1,Ph=null,zn=null,Ni=!1,To=null,mo=90,Ih=[],Lh=[],Qn=null,bo=0,Uh=null,Xa=-1,Jn=0,Ya=0,Ao=null,qa=!1;function on(){return Ce&48?Ot():Xa!==-1?Xa:Xa=Ot()}function Ai(n){if(n=n.mode,!(n&2))return 1;if(!(n&4))return as()===99?1:2;if(Jn===0&&(Jn=hs),ew.transition!==0){Ya!==0&&(Ya=Ah!==null?Ah.pendingLanes:0),n=Jn;var e=4186112&~Ya;return e&=-e,e===0&&(n=4186112&~n,e=n&-n,e===0&&(e=8192)),e}return n=as(),Ce&4&&n===98?n=Qa(12,Jn):(n=ix(n),n=Qa(n,Jn)),n}function Ri(n,e,t){if(50<bo)throw bo=0,Uh=null,Error(j(185));if(n=Pl(n,e),n===null)return null;Sl(n,e,t),n===Gt&&(ds|=e,Pt===4&&ns(n,Ft));var i=as();e===1?Ce&8&&!(Ce&48)?Nh(n):(yn(n,t),Ce===0&&(fs(),Gn())):(!(Ce&4)||i!==98&&i!==99||(Qn===null?Qn=new Set([n]):Qn.add(n)),yn(n,t)),Ah=n}function Pl(n,e){n.lanes|=e;var t=n.alternate;for(t!==null&&(t.lanes|=e),t=n,n=n.return;n!==null;)n.childLanes|=e,t=n.alternate,t!==null&&(t.childLanes|=e),t=n,n=n.return;return t.tag===3?t.stateNode:null}function yn(n,e){for(var t=n.callbackNode,i=n.suspendedLanes,r=n.pingedLanes,s=n.expirationTimes,o=n.pendingLanes;0<o;){var a=31-Li(o),l=1<<a,c=s[a];if(c===-1){if(!(l&i)||l&r){c=e,Vr(l);var u=tt;s[a]=10<=u?c+250:6<=u?c+5e3:-1}}else c<=e&&(n.expiredLanes|=l);o&=~l}if(i=No(n,n===Gt?Ft:0),e=tt,i===0)t!==null&&(t!==zu&&mh(t),n.callbackNode=null,n.callbackPriority=0);else{if(t!==null){if(n.callbackPriority===e)return;t!==zu&&mh(t)}e===15?(t=Nh.bind(null,n),Zn===null?(Zn=[t],Wa=od(bl,L0)):Zn.push(t),t=zu):e===14?t=Bo(99,Nh.bind(null,n)):(t=rx(e),t=Bo(t,t_.bind(null,n))),n.callbackPriority=e,n.callbackNode=t}}function t_(n){if(Xa=-1,Ya=Jn=0,Ce&48)throw Error(j(327));var e=n.callbackNode;if(Fi()&&n.callbackNode!==e)return null;var t=No(n,n===Gt?Ft:0);if(t===0)return null;var i=t,r=Ce;Ce|=16;var s=s_();(Gt!==n||Ft!==i)&&(fs(),is(n,i));do try{gw();break}catch(a){r_(n,a)}while(!0);if(ad(),ml.current=s,Ce=r,xt!==null?i=0:(Gt=null,Ft=0,i=Pt),hs&ds)is(n,0);else if(i!==0){if(i===2&&(Ce|=64,n.hydrate&&(n.hydrate=!1,rd(n.containerInfo)),t=u0(n),t!==0&&(i=go(n,t))),i===1)throw e=Rl,is(n,0),ns(n,t),yn(n,Ot()),e;switch(n.finishedWork=n.current.alternate,n.finishedLanes=t,i){case 0:case 1:throw Error(j(345));case 2:tr(n);break;case 3:if(ns(n,t),(t&62914560)===t&&(i=xd+500-Ot(),10<i)){if(No(n,0)!==0)break;if(r=n.suspendedLanes,(r&t)!==t){on(),n.pingedLanes|=n.suspendedLanes&r;break}n.timeoutHandle=ng(tr.bind(null,n),i);break}tr(n);break;case 4:if(ns(n,t),(t&4186112)===t)break;for(i=n.eventTimes,r=-1;0<t;){var o=31-Li(t);s=1<<o,o=i[o],o>r&&(r=o),t&=~s}if(t=r,t=Ot()-t,t=(120>t?120:480>t?480:1080>t?1080:1920>t?1920:3e3>t?3e3:4320>t?4320:1960*fw(t/1960))-t,10<t){n.timeoutHandle=ng(tr.bind(null,n),t);break}tr(n);break;case 5:tr(n);break;default:throw Error(j(329))}}return yn(n,Ot()),n.callbackNode===e?t_.bind(null,n):null}function ns(n,e){for(e&=~yd,e&=~ds,n.suspendedLanes|=e,n.pingedLanes&=~e,n=n.expirationTimes;0<e;){var t=31-Li(e),i=1<<t;n[t]=-1,e&=~i}}function Nh(n){if(Ce&48)throw Error(j(327));if(Fi(),n===Gt&&n.expiredLanes&Ft){var e=Ft,t=go(n,e);hs&ds&&(e=No(n,e),t=go(n,e))}else e=No(n,0),t=go(n,e);if(n.tag!==0&&t===2&&(Ce|=64,n.hydrate&&(n.hydrate=!1,rd(n.containerInfo)),e=u0(n),e!==0&&(t=go(n,e))),t===1)throw t=Rl,is(n,0),ns(n,e),yn(n,Ot()),t;return n.finishedWork=n.current.alternate,n.finishedLanes=e,tr(n),yn(n,Ot()),null}function pw(){if(Qn!==null){var n=Qn;Qn=null,n.forEach(function(e){e.expiredLanes|=24&e.pendingLanes,yn(e,Ot())})}Gn()}function n_(n,e){var t=Ce;Ce|=1;try{return n(e)}finally{Ce=t,Ce===0&&(fs(),Gn())}}function i_(n,e){var t=Ce;Ce&=-2,Ce|=8;try{return n(e)}finally{Ce=t,Ce===0&&(fs(),Gn())}}function Ba(n,e){dt(bh,ur),ur|=e,hs|=e}function wd(){ur=bh.current,it(bh)}function is(n,e){n.finishedWork=null,n.finishedLanes=0;var t=n.timeoutHandle;if(t!==-1&&(n.timeoutHandle=-1,$x(t)),xt!==null)for(t=xt.return;t!==null;){var i=t;switch(i.tag){case 1:i=i.type.childContextTypes,i!=null&&il();break;case 3:ls(),it(Kt),it(Bt),hd();break;case 5:ud(i);break;case 4:ls();break;case 13:it(ht);break;case 19:it(ht);break;case 10:ld(i);break;case 23:case 24:wd()}t=t.return}Gt=n,xt=Di(n.current,null),Ft=ur=hs=e,Pt=0,Rl=null,yd=ds=Yo=0}function r_(n,e){do{var t=xt;try{if(ad(),Eo.current=pl,ul){for(var i=ft.memoizedState;i!==null;){var r=i.queue;r!==null&&(r.pending=null),i=i.next}ul=!1}if(Ho=0,Rt=Dt=ft=null,Co=!1,vd.current=null,t===null||t.return===null){Pt=1,Rl=e,xt=null;break}e:{var s=n,o=t.return,a=t,l=e;if(e=Ft,a.flags|=2048,a.firstEffect=a.lastEffect=null,l!==null&&typeof l=="object"&&typeof l.then=="function"){var c=l;if(!(a.mode&2)){var u=a.alternate;u?(a.updateQueue=u.updateQueue,a.memoizedState=u.memoizedState,a.lanes=u.lanes):(a.updateQueue=null,a.memoizedState=null)}var p=(ht.current&1)!==0,f=o;do{var m;if(m=f.tag===13){var _=f.memoizedState;if(_!==null)m=_.dehydrated!==null;else{var y=f.memoizedProps;m=y.fallback===void 0?!1:y.unstable_avoidThisFallback!==!0?!0:!p}}if(m){var d=f.updateQueue;if(d===null){var h=new Set;h.add(c),f.updateQueue=h}else d.add(c);if(!(f.mode&2)){if(f.flags|=64,a.flags|=16384,a.flags&=-2981,a.tag===1)if(a.alternate===null)a.tag=17;else{var g=Ti(-1,1);g.tag=2,bi(a,g)}a.lanes|=1;break e}l=void 0,a=e;var v=s.pingCache;if(v===null?(v=s.pingCache=new lw,l=new Set,v.set(c,l)):(l=v.get(c),l===void 0&&(l=new Set,v.set(c,l))),!l.has(a)){l.add(a);var x=ww.bind(null,s,c,a);c.then(x,x)}f.flags|=4096,f.lanes=e;break e}f=f.return}while(f!==null);l=Error((Zr(a.type)||"A React component")+` suspended while rendering, but no fallback UI was specified.

Add a <Suspense fallback=...> component higher in the tree to provide a loading indicator or placeholder to display.`)}Pt!==5&&(Pt=2),l=_d(l,a),f=o;do{switch(f.tag){case 3:s=l,f.flags|=4096,e&=-e,f.lanes|=e;var R=j0(f,s,e);cg(f,R);break e;case 1:s=l;var C=f.type,T=f.stateNode;if(!(f.flags&64)&&(typeof C.getDerivedStateFromError=="function"||T!==null&&typeof T.componentDidCatch=="function"&&(zn===null||!zn.has(T)))){f.flags|=4096,e&=-e,f.lanes|=e;var L=Q0(f,s,e);cg(f,L);break e}}f=f.return}while(f!==null)}a_(t)}catch(E){e=E,xt===t&&t!==null&&(xt=t=t.return);continue}break}while(!0)}function s_(){var n=ml.current;return ml.current=pl,n===null?pl:n}function go(n,e){var t=Ce;Ce|=16;var i=s_();Gt===n&&Ft===e||is(n,e);do try{mw();break}catch(r){r_(n,r)}while(!0);if(ad(),Ce=t,ml.current=i,xt!==null)throw Error(j(261));return Gt=null,Ft=0,Pt}function mw(){for(;xt!==null;)o_(xt)}function gw(){for(;xt!==null&&!Kx();)o_(xt)}function o_(n){var e=c_(n.alternate,n,ur);n.memoizedProps=n.pendingProps,e===null?a_(n):xt=e,vd.current=null}function a_(n){var e=n;do{var t=e.alternate;if(n=e.return,e.flags&2048){if(t=aw(e),t!==null){t.flags&=2047,xt=t;return}n!==null&&(n.firstEffect=n.lastEffect=null,n.flags|=2048)}else{if(t=ow(t,e,ur),t!==null){xt=t;return}if(t=e,t.tag!==24&&t.tag!==23||t.memoizedState===null||ur&1073741824||!(t.mode&4)){for(var i=0,r=t.child;r!==null;)i|=r.lanes|r.childLanes,r=r.sibling;t.childLanes=i}n!==null&&!(n.flags&2048)&&(n.firstEffect===null&&(n.firstEffect=e.firstEffect),e.lastEffect!==null&&(n.lastEffect!==null&&(n.lastEffect.nextEffect=e.firstEffect),n.lastEffect=e.lastEffect),1<e.flags&&(n.lastEffect!==null?n.lastEffect.nextEffect=e:n.firstEffect=e,n.lastEffect=e))}if(e=e.sibling,e!==null){xt=e;return}xt=e=n}while(e!==null);Pt===0&&(Pt=5)}function tr(n){var e=as();return cr(99,_w.bind(null,n,e)),null}function _w(n,e){do Fi();while(To!==null);if(Ce&48)throw Error(j(327));var t=n.finishedWork;if(t===null)return null;if(n.finishedWork=null,n.finishedLanes=0,t===n.current)throw Error(j(177));n.callbackNode=null;var i=t.lanes|t.childLanes,r=i,s=n.pendingLanes&~r;n.pendingLanes=r,n.suspendedLanes=0,n.pingedLanes=0,n.expiredLanes&=r,n.mutableReadLanes&=r,n.entangledLanes&=r,r=n.entanglements;for(var o=n.eventTimes,a=n.expirationTimes;0<s;){var l=31-Li(s),c=1<<l;r[l]=0,o[l]=-1,a[l]=-1,s&=~c}if(Qn!==null&&!(i&24)&&Qn.has(n)&&Qn.delete(n),n===Gt&&(xt=Gt=null,Ft=0),1<t.flags?t.lastEffect!==null?(t.lastEffect.nextEffect=t,i=t.firstEffect):i=t:i=t.firstEffect,i!==null){if(r=Ce,Ce|=32,vd.current=null,Fu=za,o=Jm(),uh(o)){if("selectionStart"in o)a={start:o.selectionStart,end:o.selectionEnd};else e:if(a=(a=o.ownerDocument)&&a.defaultView||window,(c=a.getSelection&&a.getSelection())&&c.rangeCount!==0){a=c.anchorNode,s=c.anchorOffset,l=c.focusNode,c=c.focusOffset;try{a.nodeType,l.nodeType}catch{a=null;break e}var u=0,p=-1,f=-1,m=0,_=0,y=o,d=null;t:for(;;){for(var h;y!==a||s!==0&&y.nodeType!==3||(p=u+s),y!==l||c!==0&&y.nodeType!==3||(f=u+c),y.nodeType===3&&(u+=y.nodeValue.length),(h=y.firstChild)!==null;)d=y,y=h;for(;;){if(y===o)break t;if(d===a&&++m===s&&(p=u),d===l&&++_===c&&(f=u),(h=y.nextSibling)!==null)break;y=d,d=y.parentNode}y=h}a=p===-1||f===-1?null:{start:p,end:f}}else a=null;a=a||{start:0,end:0}}else a=null;Bu={focusedElem:o,selectionRange:a},za=!1,Ao=null,qa=!1,le=i;do try{vw()}catch(E){if(le===null)throw Error(j(330));Pi(le,E),le=le.nextEffect}while(le!==null);Ao=null,le=i;do try{for(o=n;le!==null;){var g=le.flags;if(g&16&&Po(le.stateNode,""),g&128){var v=le.alternate;if(v!==null){var x=v.ref;x!==null&&(typeof x=="function"?x(null):x.current=null)}}switch(g&1038){case 2:Ng(le),le.flags&=-3;break;case 6:Ng(le),le.flags&=-3,Xu(le.alternate,le);break;case 1024:le.flags&=-1025;break;case 1028:le.flags&=-1025,Xu(le.alternate,le);break;case 4:Xu(le.alternate,le);break;case 8:a=le,e_(o,a);var R=a.alternate;Lg(a),R!==null&&Lg(R)}le=le.nextEffect}}catch(E){if(le===null)throw Error(j(330));Pi(le,E),le=le.nextEffect}while(le!==null);if(x=Bu,v=Jm(),g=x.focusedElem,o=x.selectionRange,v!==g&&g&&g.ownerDocument&&v0(g.ownerDocument.documentElement,g)){for(o!==null&&uh(g)&&(v=o.start,x=o.end,x===void 0&&(x=v),"selectionStart"in g?(g.selectionStart=v,g.selectionEnd=Math.min(x,g.value.length)):(x=(v=g.ownerDocument||document)&&v.defaultView||window,x.getSelection&&(x=x.getSelection(),a=g.textContent.length,R=Math.min(o.start,a),o=o.end===void 0?R:Math.min(o.end,a),!x.extend&&R>o&&(a=o,o=R,R=a),a=Zm(g,R),s=Zm(g,o),a&&s&&(x.rangeCount!==1||x.anchorNode!==a.node||x.anchorOffset!==a.offset||x.focusNode!==s.node||x.focusOffset!==s.offset)&&(v=v.createRange(),v.setStart(a.node,a.offset),x.removeAllRanges(),R>o?(x.addRange(v),x.extend(s.node,s.offset)):(v.setEnd(s.node,s.offset),x.addRange(v)))))),v=[],x=g;x=x.parentNode;)x.nodeType===1&&v.push({element:x,left:x.scrollLeft,top:x.scrollTop});for(typeof g.focus=="function"&&g.focus(),g=0;g<v.length;g++)x=v[g],x.element.scrollLeft=x.left,x.element.scrollTop=x.top}za=!!Fu,Bu=Fu=null,n.current=t,le=i;do try{for(g=n;le!==null;){var C=le.flags;if(C&36&&hw(g,le.alternate,le),C&128){v=void 0;var T=le.ref;if(T!==null){var L=le.stateNode;switch(le.tag){case 5:v=L;break;default:v=L}typeof T=="function"?T(v):T.current=v}}le=le.nextEffect}}catch(E){if(le===null)throw Error(j(330));Pi(le,E),le=le.nextEffect}while(le!==null);le=null,Qx(),Ce=r}else n.current=t;if(Ni)Ni=!1,To=n,mo=e;else for(le=i;le!==null;)e=le.nextEffect,le.nextEffect=null,le.flags&8&&(C=le,C.sibling=null,C.stateNode=null),le=e;if(i=n.pendingLanes,i===0&&(zn=null),i===1?n===Uh?bo++:(bo=0,Uh=n):bo=0,t=t.stateNode,ar&&typeof ar.onCommitFiberRoot=="function")try{ar.onCommitFiberRoot(sd,t,void 0,(t.current.flags&64)===64)}catch{}if(yn(n,Ot()),gl)throw gl=!1,n=Ph,Ph=null,n;return Ce&8||Gn(),null}function vw(){for(;le!==null;){var n=le.alternate;qa||Ao===null||(le.flags&8?Om(le,Ao)&&(qa=!0):le.tag===13&&dw(n,le)&&Om(le,Ao)&&(qa=!0));var e=le.flags;e&256&&uw(n,le),!(e&512)||Ni||(Ni=!0,Bo(97,function(){return Fi(),null})),le=le.nextEffect}}function Fi(){if(mo!==90){var n=97<mo?97:mo;return mo=90,cr(n,xw)}return!1}function yw(n,e){Ih.push(e,n),Ni||(Ni=!0,Bo(97,function(){return Fi(),null}))}function l_(n,e){Lh.push(e,n),Ni||(Ni=!0,Bo(97,function(){return Fi(),null}))}function xw(){if(To===null)return!1;var n=To;if(To=null,Ce&48)throw Error(j(331));var e=Ce;Ce|=32;var t=Lh;Lh=[];for(var i=0;i<t.length;i+=2){var r=t[i],s=t[i+1],o=r.destroy;if(r.destroy=void 0,typeof o=="function")try{o()}catch(l){if(s===null)throw Error(j(330));Pi(s,l)}}for(t=Ih,Ih=[],i=0;i<t.length;i+=2){r=t[i],s=t[i+1];try{var a=r.create;r.destroy=a()}catch(l){if(s===null)throw Error(j(330));Pi(s,l)}}for(a=n.current.firstEffect;a!==null;)n=a.nextEffect,a.nextEffect=null,a.flags&8&&(a.sibling=null,a.stateNode=null),a=n;return Ce=e,Gn(),!0}function Og(n,e,t){e=_d(t,e),e=j0(n,e,1),bi(n,e),e=on(),n=Pl(n,1),n!==null&&(Sl(n,1,e),yn(n,e))}function Pi(n,e){if(n.tag===3)Og(n,n,e);else for(var t=n.return;t!==null;){if(t.tag===3){Og(t,n,e);break}else if(t.tag===1){var i=t.stateNode;if(typeof t.type.getDerivedStateFromError=="function"||typeof i.componentDidCatch=="function"&&(zn===null||!zn.has(i))){n=_d(e,n);var r=Q0(t,n,1);if(bi(t,r),r=on(),t=Pl(t,1),t!==null)Sl(t,1,r),yn(t,r);else if(typeof i.componentDidCatch=="function"&&(zn===null||!zn.has(i)))try{i.componentDidCatch(e,n)}catch{}break}}t=t.return}}function ww(n,e,t){var i=n.pingCache;i!==null&&i.delete(e),e=on(),n.pingedLanes|=n.suspendedLanes&t,Gt===n&&(Ft&t)===t&&(Pt===4||Pt===3&&(Ft&62914560)===Ft&&500>Ot()-xd?is(n,0):yd|=t),yn(n,e)}function Sw(n,e){var t=n.stateNode;t!==null&&t.delete(e),e=0,e===0&&(e=n.mode,e&2?e&4?(Jn===0&&(Jn=hs),e=Hr(62914560&~Jn),e===0&&(e=4194304)):e=as()===99?1:2:e=1),t=on(),n=Pl(n,e),n!==null&&(Sl(n,e,t),yn(n,t))}var c_;c_=function(n,e,t){var i=e.lanes;if(n!==null)if(n.memoizedProps!==e.pendingProps||Kt.current)An=!0;else if(t&i)An=!!(n.flags&16384);else{switch(An=!1,e.tag){case 3:Sg(e),Hu();break;case 5:fg(e);break;case 1:jt(e.type)&&Ga(e);break;case 4:vh(e,e.stateNode.containerInfo);break;case 10:i=e.memoizedProps.value;var r=e.type._context;dt(rl,r._currentValue),r._currentValue=i;break;case 13:if(e.memoizedState!==null)return t&e.child.childLanes?Mg(n,e,t):(dt(ht,ht.current&1),e=jn(n,e,t),e!==null?e.sibling:null);dt(ht,ht.current&1);break;case 19:if(i=(t&e.childLanes)!==0,n.flags&64){if(i)return Ag(n,e,t);e.flags|=64}if(r=e.memoizedState,r!==null&&(r.rendering=null,r.tail=null,r.lastEffect=null),dt(ht,ht.current),i)break;return null;case 23:case 24:return e.lanes=0,Gu(n,e,t)}return jn(n,e,t)}else An=!1;switch(e.lanes=0,e.tag){case 2:if(i=e.type,n!==null&&(n.alternate=null,e.alternate=null,e.flags|=2),n=e.pendingProps,r=os(e,Bt.current),es(e,t),r=fd(null,e,i,n,r,t),e.flags|=1,typeof r=="object"&&r!==null&&typeof r.render=="function"&&r.$$typeof===void 0){if(e.tag=1,e.memoizedState=null,e.updateQueue=null,jt(i)){var s=!0;Ga(e)}else s=!1;e.memoizedState=r.state!==null&&r.state!==void 0?r.state:null,cd(e);var o=i.getDerivedStateFromProps;typeof o=="function"&&al(e,i,o,n),r.updater=Al,e.stateNode=r,r._reactInternals=e,_h(e,i,n,t),e=Sh(null,e,i,!0,s,t)}else e.tag=0,Jt(null,e,r,t),e=e.child;return e;case 16:r=e.elementType;e:{switch(n!==null&&(n.alternate=null,e.alternate=null,e.flags|=2),n=e.pendingProps,s=r._init,r=s(r._payload),e.type=r,s=e.tag=Ew(r),n=bn(r,n),s){case 0:e=wh(null,e,r,n,t);break e;case 1:e=wg(null,e,r,n,t);break e;case 11:e=yg(null,e,r,n,t);break e;case 14:e=xg(null,e,r,bn(r.type,n),i,t);break e}throw Error(j(306,r,""))}return e;case 0:return i=e.type,r=e.pendingProps,r=e.elementType===i?r:bn(i,r),wh(n,e,i,r,t);case 1:return i=e.type,r=e.pendingProps,r=e.elementType===i?r:bn(i,r),wg(n,e,i,r,t);case 3:if(Sg(e),i=e.updateQueue,n===null||i===null)throw Error(j(282));if(i=e.pendingProps,r=e.memoizedState,r=r!==null?r.element:null,N0(n,e),ko(e,i,null,t),i=e.memoizedState.element,i===r)Hu(),e=jn(n,e,t);else{if(r=e.stateNode,(s=r.hydrate)&&(Si=Qr(e.stateNode.containerInfo.firstChild),Kn=e,s=Hn=!0),s){if(n=r.mutableSourceEagerHydrationData,n!=null)for(r=0;r<n.length;r+=2)s=n[r],s._workInProgressVersionPrimary=n[r+1],ts.push(s);for(t=B0(e,null,i,t),e.child=t;t;)t.flags=t.flags&-3|1024,t=t.sibling}else Jt(n,e,i,t),Hu();e=e.child}return e;case 5:return fg(e),n===null&&yh(e),i=e.type,r=e.pendingProps,s=n!==null?n.memoizedProps:null,o=r.children,fh(i,r)?o=null:s!==null&&fh(i,s)&&(e.flags|=16),$0(n,e),Jt(n,e,o,t),e.child;case 6:return n===null&&yh(e),null;case 13:return Mg(n,e,t);case 4:return vh(e,e.stateNode.containerInfo),i=e.pendingProps,n===null?e.child=ll(e,null,i,t):Jt(n,e,i,t),e.child;case 11:return i=e.type,r=e.pendingProps,r=e.elementType===i?r:bn(i,r),yg(n,e,i,r,t);case 7:return Jt(n,e,e.pendingProps,t),e.child;case 8:return Jt(n,e,e.pendingProps.children,t),e.child;case 12:return Jt(n,e,e.pendingProps.children,t),e.child;case 10:e:{i=e.type._context,r=e.pendingProps,o=e.memoizedProps,s=r.value;var a=e.type._context;if(dt(rl,a._currentValue),a._currentValue=s,o!==null)if(a=o.value,s=mn(a,s)?0:(typeof i._calculateChangedBits=="function"?i._calculateChangedBits(a,s):1073741823)|0,s===0){if(o.children===r.children&&!Kt.current){e=jn(n,e,t);break e}}else for(a=e.child,a!==null&&(a.return=e);a!==null;){var l=a.dependencies;if(l!==null){o=a.child;for(var c=l.firstContext;c!==null;){if(c.context===i&&c.observedBits&s){a.tag===1&&(c=Ti(-1,t&-t),c.tag=2,bi(a,c)),a.lanes|=t,c=a.alternate,c!==null&&(c.lanes|=t),U0(a.return,t),l.lanes|=t;break}c=c.next}}else o=a.tag===10&&a.type===e.type?null:a.child;if(o!==null)o.return=a;else for(o=a;o!==null;){if(o===e){o=null;break}if(a=o.sibling,a!==null){a.return=o.return,o=a;break}o=o.return}a=o}Jt(n,e,r.children,t),e=e.child}return e;case 9:return r=e.type,s=e.pendingProps,i=s.children,es(e,t),r=vn(r,s.unstable_observedBits),i=i(r),e.flags|=1,Jt(n,e,i,t),e.child;case 14:return r=e.type,s=bn(r,e.pendingProps),s=bn(r.type,s),xg(n,e,r,s,i,t);case 15:return q0(n,e,e.type,e.pendingProps,i,t);case 17:return i=e.type,r=e.pendingProps,r=e.elementType===i?r:bn(i,r),n!==null&&(n.alternate=null,e.alternate=null,e.flags|=2),e.tag=1,jt(i)?(n=!0,Ga(e)):n=!1,es(e,t),O0(e,i,r),_h(e,i,r,t),Sh(null,e,i,!0,n,t);case 19:return Ag(n,e,t);case 23:return Gu(n,e,t);case 24:return Gu(n,e,t)}throw Error(j(156,e.tag))};function Mw(n,e,t,i){this.tag=n,this.key=t,this.sibling=this.child=this.return=this.stateNode=this.type=this.elementType=null,this.index=0,this.ref=null,this.pendingProps=e,this.dependencies=this.memoizedState=this.updateQueue=this.memoizedProps=null,this.mode=i,this.flags=0,this.lastEffect=this.firstEffect=this.nextEffect=null,this.childLanes=this.lanes=0,this.alternate=null}function gn(n,e,t,i){return new Mw(n,e,t,i)}function Sd(n){return n=n.prototype,!(!n||!n.isReactComponent)}function Ew(n){if(typeof n=="function")return Sd(n)?1:0;if(n!=null){if(n=n.$$typeof,n===yl)return 11;if(n===xl)return 14}return 2}function Di(n,e){var t=n.alternate;return t===null?(t=gn(n.tag,e,n.key,n.mode),t.elementType=n.elementType,t.type=n.type,t.stateNode=n.stateNode,t.alternate=n,n.alternate=t):(t.pendingProps=e,t.type=n.type,t.flags=0,t.nextEffect=null,t.firstEffect=null,t.lastEffect=null),t.childLanes=n.childLanes,t.lanes=n.lanes,t.child=n.child,t.memoizedProps=n.memoizedProps,t.memoizedState=n.memoizedState,t.updateQueue=n.updateQueue,e=n.dependencies,t.dependencies=e===null?null:{lanes:e.lanes,firstContext:e.firstContext},t.sibling=n.sibling,t.index=n.index,t.ref=n.ref,t}function $a(n,e,t,i,r,s){var o=2;if(i=n,typeof n=="function")Sd(n)&&(o=1);else if(typeof n=="string")o=5;else e:switch(n){case yi:return rs(t.children,r,s,e);case kg:o=8,r|=16;break;case Bh:o=8,r|=1;break;case _o:return n=gn(12,t,e,r|8),n.elementType=_o,n.type=_o,n.lanes=s,n;case vo:return n=gn(13,t,e,r),n.type=vo,n.elementType=vo,n.lanes=s,n;case Za:return n=gn(19,t,e,r),n.elementType=Za,n.lanes=s,n;case Wh:return Md(t,r,s,e);case Zu:return n=gn(24,t,e,r),n.elementType=Zu,n.lanes=s,n;default:if(typeof n=="object"&&n!==null)switch(n.$$typeof){case kh:o=10;break e;case zh:o=9;break e;case yl:o=11;break e;case xl:o=14;break e;case Vh:o=16,i=null;break e;case Hh:o=22;break e}throw Error(j(130,n==null?n:typeof n,""))}return e=gn(o,t,e,r),e.elementType=n,e.type=i,e.lanes=s,e}function rs(n,e,t,i){return n=gn(7,n,i,e),n.lanes=t,n}function Md(n,e,t,i){return n=gn(23,n,i,e),n.elementType=Wh,n.lanes=t,n}function Yu(n,e,t){return n=gn(6,n,null,e),n.lanes=t,n}function qu(n,e,t){return e=gn(4,n.children!==null?n.children:[],n.key,e),e.lanes=t,e.stateNode={containerInfo:n.containerInfo,pendingChildren:null,implementation:n.implementation},e}function Cw(n,e,t){this.tag=e,this.containerInfo=n,this.finishedWork=this.pingCache=this.current=this.pendingChildren=null,this.timeoutHandle=-1,this.pendingContext=this.context=null,this.hydrate=t,this.callbackNode=null,this.callbackPriority=0,this.eventTimes=Iu(0),this.expirationTimes=Iu(-1),this.entangledLanes=this.finishedLanes=this.mutableReadLanes=this.expiredLanes=this.pingedLanes=this.suspendedLanes=this.pendingLanes=0,this.entanglements=Iu(0),this.mutableSourceEagerHydrationData=null}function Tw(n,e,t){var i=3<arguments.length&&arguments[3]!==void 0?arguments[3]:null;return{$$typeof:nr,key:i==null?null:""+i,children:n,containerInfo:e,implementation:t}}function _l(n,e,t,i){var r=e.current,s=on(),o=Ai(r);e:if(t){t=t._reactInternals;t:{if(fr(t)!==t||t.tag!==1)throw Error(j(170));var a=t;do{switch(a.tag){case 3:a=a.stateNode.context;break t;case 1:if(jt(a.type)){a=a.stateNode.__reactInternalMemoizedMergedChildContext;break t}}a=a.return}while(a!==null);throw Error(j(171))}if(t.tag===1){var l=t.type;if(jt(l)){t=T0(t,l,a);break e}}t=a}else t=Ui;return e.context===null?e.context=t:e.pendingContext=t,e=Ti(s,o),e.payload={element:n},i=i===void 0?null:i,i!==null&&(e.callback=i),bi(r,e),Ri(r,o,s),o}function $u(n){if(n=n.current,!n.child)return null;switch(n.child.tag){case 5:return n.child.stateNode;default:return n.child.stateNode}}function Fg(n,e){if(n=n.memoizedState,n!==null&&n.dehydrated!==null){var t=n.retryLane;n.retryLane=t!==0&&t<e?t:e}}function Ed(n,e){Fg(n,e),(n=n.alternate)&&Fg(n,e)}function bw(){return null}function Cd(n,e,t){var i=t!=null&&t.hydrationOptions!=null&&t.hydrationOptions.mutableSources||null;if(t=new Cw(n,e,t!=null&&t.hydrate===!0),e=gn(3,null,null,e===2?7:e===1?3:0),t.current=e,e.stateNode=t,cd(e),n[us]=t.current,w0(n.nodeType===8?n.parentNode:n),i)for(n=0;n<i.length;n++){e=i[n];var r=e._getVersion;r=r(e._source),t.mutableSourceEagerHydrationData==null?t.mutableSourceEagerHydrationData=[e,r]:t.mutableSourceEagerHydrationData.push(e,r)}this._internalRoot=t}Cd.prototype.render=function(n){_l(n,this._internalRoot,null,null)};Cd.prototype.unmount=function(){var n=this._internalRoot,e=n.containerInfo;_l(null,n,null,function(){e[us]=null})};function qo(n){return!(!n||n.nodeType!==1&&n.nodeType!==9&&n.nodeType!==11&&(n.nodeType!==8||n.nodeValue!==" react-mount-point-unstable "))}function Aw(n,e){if(e||(e=n?n.nodeType===9?n.documentElement:n.firstChild:null,e=!(!e||e.nodeType!==1||!e.hasAttribute("data-reactroot"))),!e)for(var t;t=n.lastChild;)n.removeChild(t);return new Cd(n,0,e?{hydrate:!0}:void 0)}function Il(n,e,t,i,r){var s=t._reactRootContainer;if(s){var o=s._internalRoot;if(typeof r=="function"){var a=r;r=function(){var c=$u(o);a.call(c)}}_l(e,o,n,r)}else{if(s=t._reactRootContainer=Aw(t,i),o=s._internalRoot,typeof r=="function"){var l=r;r=function(){var c=$u(o);l.call(c)}}i_(function(){_l(e,o,n,r)})}return $u(o)}e0=function(n){if(n.tag===13){var e=on();Ri(n,4,e),Ed(n,4)}};Zh=function(n){if(n.tag===13){var e=on();Ri(n,67108864,e),Ed(n,67108864)}};t0=function(n){if(n.tag===13){var e=on(),t=Ai(n);Ri(n,t,e),Ed(n,t)}};n0=function(n,e){return e()};sh=function(n,e,t){switch(e){case"input":if(Ku(n,t),e=t.name,t.type==="radio"&&e!=null){for(t=n;t.parentNode;)t=t.parentNode;for(t=t.querySelectorAll("input[name="+JSON.stringify(""+e)+'][type="radio"]'),e=0;e<t.length;e++){var i=t[e];if(i!==n&&i.form===n.form){var r=Tl(i);if(!r)throw Error(j(90));Vg(i),Ku(i,r)}}}break;case"textarea":Gg(n,t);break;case"select":e=t.value,e!=null&&Jr(n,!!t.multiple,e,!1)}};Yh=n_;Jg=function(n,e,t,i,r){var s=Ce;Ce|=4;try{return cr(98,n.bind(null,e,t,i,r))}finally{Ce=s,Ce===0&&(fs(),Gn())}};qh=function(){!(Ce&49)&&(pw(),Fi())};Kg=function(n,e){var t=Ce;Ce|=2;try{return n(e)}finally{Ce=t,Ce===0&&(fs(),Gn())}};function u_(n,e){var t=2<arguments.length&&arguments[2]!==void 0?arguments[2]:null;if(!qo(e))throw Error(j(200));return Tw(n,e,null,t)}var Rw={Events:[Wo,Yr,Tl,$g,Zg,Fi,{current:!1}]},co={findFiberByHostInstance:rr,bundleType:0,version:"17.0.2",rendererPackageName:"react-dom"},Pw={bundleType:co.bundleType,version:co.version,rendererPackageName:co.rendererPackageName,rendererConfig:co.rendererConfig,overrideHookState:null,overrideHookStateDeletePath:null,overrideHookStateRenamePath:null,overrideProps:null,overridePropsDeletePath:null,overridePropsRenamePath:null,setSuspenseHandler:null,scheduleUpdate:null,currentDispatcherRef:dr.ReactCurrentDispatcher,findHostInstanceByFiber:function(n){return n=Qg(n),n===null?null:n.stateNode},findFiberByHostInstance:co.findFiberByHostInstance||bw,findHostInstancesForRefresh:null,scheduleRefresh:null,scheduleRoot:null,setRefreshHandler:null,getCurrentFiber:null};if(typeof __REACT_DEVTOOLS_GLOBAL_HOOK__<"u"&&(uo=__REACT_DEVTOOLS_GLOBAL_HOOK__,!uo.isDisabled&&uo.supportsFiber))try{sd=uo.inject(Pw),ar=uo}catch{}var uo;xn.__SECRET_INTERNALS_DO_NOT_USE_OR_YOU_WILL_BE_FIRED=Rw;xn.createPortal=u_;xn.findDOMNode=function(n){if(n==null)return null;if(n.nodeType===1)return n;var e=n._reactInternals;if(e===void 0)throw typeof n.render=="function"?Error(j(188)):Error(j(268,Object.keys(n)));return n=Qg(e),n=n===null?null:n.stateNode,n};xn.flushSync=function(n,e){var t=Ce;if(t&48)return n(e);Ce|=1;try{if(n)return cr(99,n.bind(null,e))}finally{Ce=t,Gn()}};xn.hydrate=function(n,e,t){if(!qo(e))throw Error(j(200));return Il(null,n,e,!0,t)};xn.render=function(n,e,t){if(!qo(e))throw Error(j(200));return Il(null,n,e,!1,t)};xn.unmountComponentAtNode=function(n){if(!qo(n))throw Error(j(40));return n._reactRootContainer?(i_(function(){Il(null,null,n,!1,function(){n._reactRootContainer=null,n[us]=null})}),!0):!1};xn.unstable_batchedUpdates=n_;xn.unstable_createPortal=function(n,e){return u_(n,e,2<arguments.length&&arguments[2]!==void 0?arguments[2]:null)};xn.unstable_renderSubtreeIntoContainer=function(n,e,t,i){if(!qo(t))throw Error(j(200));if(n==null||n._reactInternals===void 0)throw Error(j(38));return Il(n,e,t,!1,i)};xn.version="17.0.2"});var p_=Qi((_b,f_)=>{"use strict";function d_(){if(!(typeof __REACT_DEVTOOLS_GLOBAL_HOOK__>"u"||typeof __REACT_DEVTOOLS_GLOBAL_HOOK__.checkDCE!="function"))try{__REACT_DEVTOOLS_GLOBAL_HOOK__.checkDCE(d_)}catch(n){console.error(n)}}d_(),f_.exports=h_()});var En=jp(mu()),my=jp(p_());var dp="166";var Iw=0,m_=1,Lw=2;var mv=1,fp=2,ai=3,$i=0,tn=1,li=2,Yi=0,Us=1,g_=2,__=3,v_=4,Uw=5,Sr=100,Nw=101,Dw=102,Ow=103,Fw=104,Bw=200,kw=201,zw=202,Vw=203,tf=204,nf=205,Hw=206,Gw=207,Ww=208,Xw=209,Yw=210,qw=211,$w=212,Zw=213,Jw=214,Kw=0,jw=1,Qw=2,oc=3,eS=4,tS=5,nS=6,iS=7,gv=0,rS=1,sS=2,qi=0,oS=1,aS=2,lS=3,cS=4,uS=5,hS=6,dS=7;var _v=300,Fs=301,Bs=302,rf=303,sf=304,Lc=306,of=1e3,Er=1001,af=1002,Sn=1003,fS=1004;var Ll=1005;var qt=1006,Td=1007;var Cr=1008;var di=1009,vv=1010,yv=1011,ta=1012,pp=1013,Tr=1014,ci=1015,aa=1016,mp=1017,gp=1018,ks=1020,xv=35902,wv=1021,Sv=1022,Un=1023,Mv=1024,Ev=1025,Ns=1026,zs=1027,Cv=1028,_p=1029,Tv=1030,vp=1031;var yp=1033,tc=33776,nc=33777,ic=33778,rc=33779,lf=35840,cf=35841,uf=35842,hf=35843,df=36196,ff=37492,pf=37496,mf=37808,gf=37809,_f=37810,vf=37811,yf=37812,xf=37813,wf=37814,Sf=37815,Mf=37816,Ef=37817,Cf=37818,Tf=37819,bf=37820,Af=37821,sc=36492,Rf=36494,Pf=36495,bv=36283,If=36284,Lf=36285,Uf=36286;var ac=2300,Nf=2301,bd=2302,y_=2400,x_=2401,w_=2402;var pS=3200,mS=3201,Av=0,gS=1,Wi="",un="srgb",Ji="srgb-linear",xp="display-p3",Uc="display-p3-linear",lc="linear",rt="srgb",cc="rec709",uc="p3";var ps=7680;var S_=519,_S=512,vS=513,yS=514,Rv=515,xS=516,wS=517,SS=518,MS=519,M_=35044;var E_="300 es",ui=2e3,hc=2001,Zi=class{addEventListener(e,t){this._listeners===void 0&&(this._listeners={});let i=this._listeners;i[e]===void 0&&(i[e]=[]),i[e].indexOf(t)===-1&&i[e].push(t)}hasEventListener(e,t){if(this._listeners===void 0)return!1;let i=this._listeners;return i[e]!==void 0&&i[e].indexOf(t)!==-1}removeEventListener(e,t){if(this._listeners===void 0)return;let r=this._listeners[e];if(r!==void 0){let s=r.indexOf(t);s!==-1&&r.splice(s,1)}}dispatchEvent(e){if(this._listeners===void 0)return;let i=this._listeners[e.type];if(i!==void 0){e.target=this;let r=i.slice(0);for(let s=0,o=r.length;s<o;s++)r[s].call(this,e);e.target=null}}},kt=["00","01","02","03","04","05","06","07","08","09","0a","0b","0c","0d","0e","0f","10","11","12","13","14","15","16","17","18","19","1a","1b","1c","1d","1e","1f","20","21","22","23","24","25","26","27","28","29","2a","2b","2c","2d","2e","2f","30","31","32","33","34","35","36","37","38","39","3a","3b","3c","3d","3e","3f","40","41","42","43","44","45","46","47","48","49","4a","4b","4c","4d","4e","4f","50","51","52","53","54","55","56","57","58","59","5a","5b","5c","5d","5e","5f","60","61","62","63","64","65","66","67","68","69","6a","6b","6c","6d","6e","6f","70","71","72","73","74","75","76","77","78","79","7a","7b","7c","7d","7e","7f","80","81","82","83","84","85","86","87","88","89","8a","8b","8c","8d","8e","8f","90","91","92","93","94","95","96","97","98","99","9a","9b","9c","9d","9e","9f","a0","a1","a2","a3","a4","a5","a6","a7","a8","a9","aa","ab","ac","ad","ae","af","b0","b1","b2","b3","b4","b5","b6","b7","b8","b9","ba","bb","bc","bd","be","bf","c0","c1","c2","c3","c4","c5","c6","c7","c8","c9","ca","cb","cc","cd","ce","cf","d0","d1","d2","d3","d4","d5","d6","d7","d8","d9","da","db","dc","dd","de","df","e0","e1","e2","e3","e4","e5","e6","e7","e8","e9","ea","eb","ec","ed","ee","ef","f0","f1","f2","f3","f4","f5","f6","f7","f8","f9","fa","fb","fc","fd","fe","ff"];var Ad=Math.PI/180,Df=180/Math.PI;function la(){let n=Math.random()*4294967295|0,e=Math.random()*4294967295|0,t=Math.random()*4294967295|0,i=Math.random()*4294967295|0;return(kt[n&255]+kt[n>>8&255]+kt[n>>16&255]+kt[n>>24&255]+"-"+kt[e&255]+kt[e>>8&255]+"-"+kt[e>>16&15|64]+kt[e>>24&255]+"-"+kt[t&63|128]+kt[t>>8&255]+"-"+kt[t>>16&255]+kt[t>>24&255]+kt[i&255]+kt[i>>8&255]+kt[i>>16&255]+kt[i>>24&255]).toLowerCase()}function en(n,e,t){return Math.max(e,Math.min(t,n))}function ES(n,e){return(n%e+e)%e}function Rd(n,e,t){return(1-t)*n+t*e}function $o(n,e){switch(e.constructor){case Float32Array:return n;case Uint32Array:return n/4294967295;case Uint16Array:return n/65535;case Uint8Array:return n/255;case Int32Array:return Math.max(n/2147483647,-1);case Int16Array:return Math.max(n/32767,-1);case Int8Array:return Math.max(n/127,-1);default:throw new Error("Invalid component type.")}}function Qt(n,e){switch(e.constructor){case Float32Array:return n;case Uint32Array:return Math.round(n*4294967295);case Uint16Array:return Math.round(n*65535);case Uint8Array:return Math.round(n*255);case Int32Array:return Math.round(n*2147483647);case Int16Array:return Math.round(n*32767);case Int8Array:return Math.round(n*127);default:throw new Error("Invalid component type.")}}var ze=class n{constructor(e=0,t=0){n.prototype.isVector2=!0,this.x=e,this.y=t}get width(){return this.x}set width(e){this.x=e}get height(){return this.y}set height(e){this.y=e}set(e,t){return this.x=e,this.y=t,this}setScalar(e){return this.x=e,this.y=e,this}setX(e){return this.x=e,this}setY(e){return this.y=e,this}setComponent(e,t){switch(e){case 0:this.x=t;break;case 1:this.y=t;break;default:throw new Error("index is out of range: "+e)}return this}getComponent(e){switch(e){case 0:return this.x;case 1:return this.y;default:throw new Error("index is out of range: "+e)}}clone(){return new this.constructor(this.x,this.y)}copy(e){return this.x=e.x,this.y=e.y,this}add(e){return this.x+=e.x,this.y+=e.y,this}addScalar(e){return this.x+=e,this.y+=e,this}addVectors(e,t){return this.x=e.x+t.x,this.y=e.y+t.y,this}addScaledVector(e,t){return this.x+=e.x*t,this.y+=e.y*t,this}sub(e){return this.x-=e.x,this.y-=e.y,this}subScalar(e){return this.x-=e,this.y-=e,this}subVectors(e,t){return this.x=e.x-t.x,this.y=e.y-t.y,this}multiply(e){return this.x*=e.x,this.y*=e.y,this}multiplyScalar(e){return this.x*=e,this.y*=e,this}divide(e){return this.x/=e.x,this.y/=e.y,this}divideScalar(e){return this.multiplyScalar(1/e)}applyMatrix3(e){let t=this.x,i=this.y,r=e.elements;return this.x=r[0]*t+r[3]*i+r[6],this.y=r[1]*t+r[4]*i+r[7],this}min(e){return this.x=Math.min(this.x,e.x),this.y=Math.min(this.y,e.y),this}max(e){return this.x=Math.max(this.x,e.x),this.y=Math.max(this.y,e.y),this}clamp(e,t){return this.x=Math.max(e.x,Math.min(t.x,this.x)),this.y=Math.max(e.y,Math.min(t.y,this.y)),this}clampScalar(e,t){return this.x=Math.max(e,Math.min(t,this.x)),this.y=Math.max(e,Math.min(t,this.y)),this}clampLength(e,t){let i=this.length();return this.divideScalar(i||1).multiplyScalar(Math.max(e,Math.min(t,i)))}floor(){return this.x=Math.floor(this.x),this.y=Math.floor(this.y),this}ceil(){return this.x=Math.ceil(this.x),this.y=Math.ceil(this.y),this}round(){return this.x=Math.round(this.x),this.y=Math.round(this.y),this}roundToZero(){return this.x=Math.trunc(this.x),this.y=Math.trunc(this.y),this}negate(){return this.x=-this.x,this.y=-this.y,this}dot(e){return this.x*e.x+this.y*e.y}cross(e){return this.x*e.y-this.y*e.x}lengthSq(){return this.x*this.x+this.y*this.y}length(){return Math.sqrt(this.x*this.x+this.y*this.y)}manhattanLength(){return Math.abs(this.x)+Math.abs(this.y)}normalize(){return this.divideScalar(this.length()||1)}angle(){return Math.atan2(-this.y,-this.x)+Math.PI}angleTo(e){let t=Math.sqrt(this.lengthSq()*e.lengthSq());if(t===0)return Math.PI/2;let i=this.dot(e)/t;return Math.acos(en(i,-1,1))}distanceTo(e){return Math.sqrt(this.distanceToSquared(e))}distanceToSquared(e){let t=this.x-e.x,i=this.y-e.y;return t*t+i*i}manhattanDistanceTo(e){return Math.abs(this.x-e.x)+Math.abs(this.y-e.y)}setLength(e){return this.normalize().multiplyScalar(e)}lerp(e,t){return this.x+=(e.x-this.x)*t,this.y+=(e.y-this.y)*t,this}lerpVectors(e,t,i){return this.x=e.x+(t.x-e.x)*i,this.y=e.y+(t.y-e.y)*i,this}equals(e){return e.x===this.x&&e.y===this.y}fromArray(e,t=0){return this.x=e[t],this.y=e[t+1],this}toArray(e=[],t=0){return e[t]=this.x,e[t+1]=this.y,e}fromBufferAttribute(e,t){return this.x=e.getX(t),this.y=e.getY(t),this}rotateAround(e,t){let i=Math.cos(t),r=Math.sin(t),s=this.x-e.x,o=this.y-e.y;return this.x=s*i-o*r+e.x,this.y=s*r+o*i+e.y,this}random(){return this.x=Math.random(),this.y=Math.random(),this}*[Symbol.iterator](){yield this.x,yield this.y}},Ue=class n{constructor(e,t,i,r,s,o,a,l,c){n.prototype.isMatrix3=!0,this.elements=[1,0,0,0,1,0,0,0,1],e!==void 0&&this.set(e,t,i,r,s,o,a,l,c)}set(e,t,i,r,s,o,a,l,c){let u=this.elements;return u[0]=e,u[1]=r,u[2]=a,u[3]=t,u[4]=s,u[5]=l,u[6]=i,u[7]=o,u[8]=c,this}identity(){return this.set(1,0,0,0,1,0,0,0,1),this}copy(e){let t=this.elements,i=e.elements;return t[0]=i[0],t[1]=i[1],t[2]=i[2],t[3]=i[3],t[4]=i[4],t[5]=i[5],t[6]=i[6],t[7]=i[7],t[8]=i[8],this}extractBasis(e,t,i){return e.setFromMatrix3Column(this,0),t.setFromMatrix3Column(this,1),i.setFromMatrix3Column(this,2),this}setFromMatrix4(e){let t=e.elements;return this.set(t[0],t[4],t[8],t[1],t[5],t[9],t[2],t[6],t[10]),this}multiply(e){return this.multiplyMatrices(this,e)}premultiply(e){return this.multiplyMatrices(e,this)}multiplyMatrices(e,t){let i=e.elements,r=t.elements,s=this.elements,o=i[0],a=i[3],l=i[6],c=i[1],u=i[4],p=i[7],f=i[2],m=i[5],_=i[8],y=r[0],d=r[3],h=r[6],g=r[1],v=r[4],x=r[7],R=r[2],C=r[5],T=r[8];return s[0]=o*y+a*g+l*R,s[3]=o*d+a*v+l*C,s[6]=o*h+a*x+l*T,s[1]=c*y+u*g+p*R,s[4]=c*d+u*v+p*C,s[7]=c*h+u*x+p*T,s[2]=f*y+m*g+_*R,s[5]=f*d+m*v+_*C,s[8]=f*h+m*x+_*T,this}multiplyScalar(e){let t=this.elements;return t[0]*=e,t[3]*=e,t[6]*=e,t[1]*=e,t[4]*=e,t[7]*=e,t[2]*=e,t[5]*=e,t[8]*=e,this}determinant(){let e=this.elements,t=e[0],i=e[1],r=e[2],s=e[3],o=e[4],a=e[5],l=e[6],c=e[7],u=e[8];return t*o*u-t*a*c-i*s*u+i*a*l+r*s*c-r*o*l}invert(){let e=this.elements,t=e[0],i=e[1],r=e[2],s=e[3],o=e[4],a=e[5],l=e[6],c=e[7],u=e[8],p=u*o-a*c,f=a*l-u*s,m=c*s-o*l,_=t*p+i*f+r*m;if(_===0)return this.set(0,0,0,0,0,0,0,0,0);let y=1/_;return e[0]=p*y,e[1]=(r*c-u*i)*y,e[2]=(a*i-r*o)*y,e[3]=f*y,e[4]=(u*t-r*l)*y,e[5]=(r*s-a*t)*y,e[6]=m*y,e[7]=(i*l-c*t)*y,e[8]=(o*t-i*s)*y,this}transpose(){let e,t=this.elements;return e=t[1],t[1]=t[3],t[3]=e,e=t[2],t[2]=t[6],t[6]=e,e=t[5],t[5]=t[7],t[7]=e,this}getNormalMatrix(e){return this.setFromMatrix4(e).invert().transpose()}transposeIntoArray(e){let t=this.elements;return e[0]=t[0],e[1]=t[3],e[2]=t[6],e[3]=t[1],e[4]=t[4],e[5]=t[7],e[6]=t[2],e[7]=t[5],e[8]=t[8],this}setUvTransform(e,t,i,r,s,o,a){let l=Math.cos(s),c=Math.sin(s);return this.set(i*l,i*c,-i*(l*o+c*a)+o+e,-r*c,r*l,-r*(-c*o+l*a)+a+t,0,0,1),this}scale(e,t){return this.premultiply(Pd.makeScale(e,t)),this}rotate(e){return this.premultiply(Pd.makeRotation(-e)),this}translate(e,t){return this.premultiply(Pd.makeTranslation(e,t)),this}makeTranslation(e,t){return e.isVector2?this.set(1,0,e.x,0,1,e.y,0,0,1):this.set(1,0,e,0,1,t,0,0,1),this}makeRotation(e){let t=Math.cos(e),i=Math.sin(e);return this.set(t,-i,0,i,t,0,0,0,1),this}makeScale(e,t){return this.set(e,0,0,0,t,0,0,0,1),this}equals(e){let t=this.elements,i=e.elements;for(let r=0;r<9;r++)if(t[r]!==i[r])return!1;return!0}fromArray(e,t=0){for(let i=0;i<9;i++)this.elements[i]=e[i+t];return this}toArray(e=[],t=0){let i=this.elements;return e[t]=i[0],e[t+1]=i[1],e[t+2]=i[2],e[t+3]=i[3],e[t+4]=i[4],e[t+5]=i[5],e[t+6]=i[6],e[t+7]=i[7],e[t+8]=i[8],e}clone(){return new this.constructor().fromArray(this.elements)}},Pd=new Ue;function Pv(n){for(let e=n.length-1;e>=0;--e)if(n[e]>=65535)return!0;return!1}function na(n){return document.createElementNS("http://www.w3.org/1999/xhtml",n)}function CS(){let n=na("canvas");return n.style.display="block",n}var C_={};function Iv(n){n in C_||(C_[n]=!0,console.warn(n))}function TS(n,e,t){return new Promise(function(i,r){function s(){switch(n.clientWaitSync(e,n.SYNC_FLUSH_COMMANDS_BIT,0)){case n.WAIT_FAILED:r();break;case n.TIMEOUT_EXPIRED:setTimeout(s,t);break;default:i()}}setTimeout(s,t)})}var T_=new Ue().set(.8224621,.177538,0,.0331941,.9668058,0,.0170827,.0723974,.9105199),b_=new Ue().set(1.2249401,-.2249404,0,-.0420569,1.0420571,0,-.0196376,-.0786361,1.0982735),Ul={[Ji]:{transfer:lc,primaries:cc,toReference:n=>n,fromReference:n=>n},[un]:{transfer:rt,primaries:cc,toReference:n=>n.convertSRGBToLinear(),fromReference:n=>n.convertLinearToSRGB()},[Uc]:{transfer:lc,primaries:uc,toReference:n=>n.applyMatrix3(b_),fromReference:n=>n.applyMatrix3(T_)},[xp]:{transfer:rt,primaries:uc,toReference:n=>n.convertSRGBToLinear().applyMatrix3(b_),fromReference:n=>n.applyMatrix3(T_).convertLinearToSRGB()}},bS=new Set([Ji,Uc]),Ke={enabled:!0,_workingColorSpace:Ji,get workingColorSpace(){return this._workingColorSpace},set workingColorSpace(n){if(!bS.has(n))throw new Error(`Unsupported working color space, "${n}".`);this._workingColorSpace=n},convert:function(n,e,t){if(this.enabled===!1||e===t||!e||!t)return n;let i=Ul[e].toReference,r=Ul[t].fromReference;return r(i(n))},fromWorkingColorSpace:function(n,e){return this.convert(n,this._workingColorSpace,e)},toWorkingColorSpace:function(n,e){return this.convert(n,e,this._workingColorSpace)},getPrimaries:function(n){return Ul[n].primaries},getTransfer:function(n){return n===Wi?lc:Ul[n].transfer}};function Ds(n){return n<.04045?n*.0773993808:Math.pow(n*.9478672986+.0521327014,2.4)}function Id(n){return n<.0031308?n*12.92:1.055*Math.pow(n,.41666)-.055}var ms,Of=class{static getDataURL(e){if(/^data:/i.test(e.src)||typeof HTMLCanvasElement>"u")return e.src;let t;if(e instanceof HTMLCanvasElement)t=e;else{ms===void 0&&(ms=na("canvas")),ms.width=e.width,ms.height=e.height;let i=ms.getContext("2d");e instanceof ImageData?i.putImageData(e,0,0):i.drawImage(e,0,0,e.width,e.height),t=ms}return t.width>2048||t.height>2048?(console.warn("THREE.ImageUtils.getDataURL: Image converted to jpg for performance reasons",e),t.toDataURL("image/jpeg",.6)):t.toDataURL("image/png")}static sRGBToLinear(e){if(typeof HTMLImageElement<"u"&&e instanceof HTMLImageElement||typeof HTMLCanvasElement<"u"&&e instanceof HTMLCanvasElement||typeof ImageBitmap<"u"&&e instanceof ImageBitmap){let t=na("canvas");t.width=e.width,t.height=e.height;let i=t.getContext("2d");i.drawImage(e,0,0,e.width,e.height);let r=i.getImageData(0,0,e.width,e.height),s=r.data;for(let o=0;o<s.length;o++)s[o]=Ds(s[o]/255)*255;return i.putImageData(r,0,0),t}else if(e.data){let t=e.data.slice(0);for(let i=0;i<t.length;i++)t instanceof Uint8Array||t instanceof Uint8ClampedArray?t[i]=Math.floor(Ds(t[i]/255)*255):t[i]=Ds(t[i]);return{data:t,width:e.width,height:e.height}}else return console.warn("THREE.ImageUtils.sRGBToLinear(): Unsupported image type. No color space conversion applied."),e}},AS=0,dc=class{constructor(e=null){this.isSource=!0,Object.defineProperty(this,"id",{value:AS++}),this.uuid=la(),this.data=e,this.dataReady=!0,this.version=0}set needsUpdate(e){e===!0&&this.version++}toJSON(e){let t=e===void 0||typeof e=="string";if(!t&&e.images[this.uuid]!==void 0)return e.images[this.uuid];let i={uuid:this.uuid,url:""},r=this.data;if(r!==null){let s;if(Array.isArray(r)){s=[];for(let o=0,a=r.length;o<a;o++)r[o].isDataTexture?s.push(Ld(r[o].image)):s.push(Ld(r[o]))}else s=Ld(r);i.url=s}return t||(e.images[this.uuid]=i),i}};function Ld(n){return typeof HTMLImageElement<"u"&&n instanceof HTMLImageElement||typeof HTMLCanvasElement<"u"&&n instanceof HTMLCanvasElement||typeof ImageBitmap<"u"&&n instanceof ImageBitmap?Of.getDataURL(n):n.data?{data:Array.from(n.data),width:n.width,height:n.height,type:n.data.constructor.name}:(console.warn("THREE.Texture: Unable to serialize Texture."),{})}var RS=0,$t=class n extends Zi{constructor(e=n.DEFAULT_IMAGE,t=n.DEFAULT_MAPPING,i=Er,r=Er,s=qt,o=Cr,a=Un,l=di,c=n.DEFAULT_ANISOTROPY,u=Wi){super(),this.isTexture=!0,Object.defineProperty(this,"id",{value:RS++}),this.uuid=la(),this.name="",this.source=new dc(e),this.mipmaps=[],this.mapping=t,this.channel=0,this.wrapS=i,this.wrapT=r,this.magFilter=s,this.minFilter=o,this.anisotropy=c,this.format=a,this.internalFormat=null,this.type=l,this.offset=new ze(0,0),this.repeat=new ze(1,1),this.center=new ze(0,0),this.rotation=0,this.matrixAutoUpdate=!0,this.matrix=new Ue,this.generateMipmaps=!0,this.premultiplyAlpha=!1,this.flipY=!0,this.unpackAlignment=4,this.colorSpace=u,this.userData={},this.version=0,this.onUpdate=null,this.isRenderTargetTexture=!1,this.pmremVersion=0}get image(){return this.source.data}set image(e=null){this.source.data=e}updateMatrix(){this.matrix.setUvTransform(this.offset.x,this.offset.y,this.repeat.x,this.repeat.y,this.rotation,this.center.x,this.center.y)}clone(){return new this.constructor().copy(this)}copy(e){return this.name=e.name,this.source=e.source,this.mipmaps=e.mipmaps.slice(0),this.mapping=e.mapping,this.channel=e.channel,this.wrapS=e.wrapS,this.wrapT=e.wrapT,this.magFilter=e.magFilter,this.minFilter=e.minFilter,this.anisotropy=e.anisotropy,this.format=e.format,this.internalFormat=e.internalFormat,this.type=e.type,this.offset.copy(e.offset),this.repeat.copy(e.repeat),this.center.copy(e.center),this.rotation=e.rotation,this.matrixAutoUpdate=e.matrixAutoUpdate,this.matrix.copy(e.matrix),this.generateMipmaps=e.generateMipmaps,this.premultiplyAlpha=e.premultiplyAlpha,this.flipY=e.flipY,this.unpackAlignment=e.unpackAlignment,this.colorSpace=e.colorSpace,this.userData=JSON.parse(JSON.stringify(e.userData)),this.needsUpdate=!0,this}toJSON(e){let t=e===void 0||typeof e=="string";if(!t&&e.textures[this.uuid]!==void 0)return e.textures[this.uuid];let i={metadata:{version:4.6,type:"Texture",generator:"Texture.toJSON"},uuid:this.uuid,name:this.name,image:this.source.toJSON(e).uuid,mapping:this.mapping,channel:this.channel,repeat:[this.repeat.x,this.repeat.y],offset:[this.offset.x,this.offset.y],center:[this.center.x,this.center.y],rotation:this.rotation,wrap:[this.wrapS,this.wrapT],format:this.format,internalFormat:this.internalFormat,type:this.type,colorSpace:this.colorSpace,minFilter:this.minFilter,magFilter:this.magFilter,anisotropy:this.anisotropy,flipY:this.flipY,generateMipmaps:this.generateMipmaps,premultiplyAlpha:this.premultiplyAlpha,unpackAlignment:this.unpackAlignment};return Object.keys(this.userData).length>0&&(i.userData=this.userData),t||(e.textures[this.uuid]=i),i}dispose(){this.dispatchEvent({type:"dispose"})}transformUv(e){if(this.mapping!==_v)return e;if(e.applyMatrix3(this.matrix),e.x<0||e.x>1)switch(this.wrapS){case of:e.x=e.x-Math.floor(e.x);break;case Er:e.x=e.x<0?0:1;break;case af:Math.abs(Math.floor(e.x)%2)===1?e.x=Math.ceil(e.x)-e.x:e.x=e.x-Math.floor(e.x);break}if(e.y<0||e.y>1)switch(this.wrapT){case of:e.y=e.y-Math.floor(e.y);break;case Er:e.y=e.y<0?0:1;break;case af:Math.abs(Math.floor(e.y)%2)===1?e.y=Math.ceil(e.y)-e.y:e.y=e.y-Math.floor(e.y);break}return this.flipY&&(e.y=1-e.y),e}set needsUpdate(e){e===!0&&(this.version++,this.source.needsUpdate=!0)}set needsPMREMUpdate(e){e===!0&&this.pmremVersion++}};$t.DEFAULT_IMAGE=null;$t.DEFAULT_MAPPING=_v;$t.DEFAULT_ANISOTROPY=1;var wt=class n{constructor(e=0,t=0,i=0,r=1){n.prototype.isVector4=!0,this.x=e,this.y=t,this.z=i,this.w=r}get width(){return this.z}set width(e){this.z=e}get height(){return this.w}set height(e){this.w=e}set(e,t,i,r){return this.x=e,this.y=t,this.z=i,this.w=r,this}setScalar(e){return this.x=e,this.y=e,this.z=e,this.w=e,this}setX(e){return this.x=e,this}setY(e){return this.y=e,this}setZ(e){return this.z=e,this}setW(e){return this.w=e,this}setComponent(e,t){switch(e){case 0:this.x=t;break;case 1:this.y=t;break;case 2:this.z=t;break;case 3:this.w=t;break;default:throw new Error("index is out of range: "+e)}return this}getComponent(e){switch(e){case 0:return this.x;case 1:return this.y;case 2:return this.z;case 3:return this.w;default:throw new Error("index is out of range: "+e)}}clone(){return new this.constructor(this.x,this.y,this.z,this.w)}copy(e){return this.x=e.x,this.y=e.y,this.z=e.z,this.w=e.w!==void 0?e.w:1,this}add(e){return this.x+=e.x,this.y+=e.y,this.z+=e.z,this.w+=e.w,this}addScalar(e){return this.x+=e,this.y+=e,this.z+=e,this.w+=e,this}addVectors(e,t){return this.x=e.x+t.x,this.y=e.y+t.y,this.z=e.z+t.z,this.w=e.w+t.w,this}addScaledVector(e,t){return this.x+=e.x*t,this.y+=e.y*t,this.z+=e.z*t,this.w+=e.w*t,this}sub(e){return this.x-=e.x,this.y-=e.y,this.z-=e.z,this.w-=e.w,this}subScalar(e){return this.x-=e,this.y-=e,this.z-=e,this.w-=e,this}subVectors(e,t){return this.x=e.x-t.x,this.y=e.y-t.y,this.z=e.z-t.z,this.w=e.w-t.w,this}multiply(e){return this.x*=e.x,this.y*=e.y,this.z*=e.z,this.w*=e.w,this}multiplyScalar(e){return this.x*=e,this.y*=e,this.z*=e,this.w*=e,this}applyMatrix4(e){let t=this.x,i=this.y,r=this.z,s=this.w,o=e.elements;return this.x=o[0]*t+o[4]*i+o[8]*r+o[12]*s,this.y=o[1]*t+o[5]*i+o[9]*r+o[13]*s,this.z=o[2]*t+o[6]*i+o[10]*r+o[14]*s,this.w=o[3]*t+o[7]*i+o[11]*r+o[15]*s,this}divideScalar(e){return this.multiplyScalar(1/e)}setAxisAngleFromQuaternion(e){this.w=2*Math.acos(e.w);let t=Math.sqrt(1-e.w*e.w);return t<1e-4?(this.x=1,this.y=0,this.z=0):(this.x=e.x/t,this.y=e.y/t,this.z=e.z/t),this}setAxisAngleFromRotationMatrix(e){let t,i,r,s,l=e.elements,c=l[0],u=l[4],p=l[8],f=l[1],m=l[5],_=l[9],y=l[2],d=l[6],h=l[10];if(Math.abs(u-f)<.01&&Math.abs(p-y)<.01&&Math.abs(_-d)<.01){if(Math.abs(u+f)<.1&&Math.abs(p+y)<.1&&Math.abs(_+d)<.1&&Math.abs(c+m+h-3)<.1)return this.set(1,0,0,0),this;t=Math.PI;let v=(c+1)/2,x=(m+1)/2,R=(h+1)/2,C=(u+f)/4,T=(p+y)/4,L=(_+d)/4;return v>x&&v>R?v<.01?(i=0,r=.707106781,s=.707106781):(i=Math.sqrt(v),r=C/i,s=T/i):x>R?x<.01?(i=.707106781,r=0,s=.707106781):(r=Math.sqrt(x),i=C/r,s=L/r):R<.01?(i=.707106781,r=.707106781,s=0):(s=Math.sqrt(R),i=T/s,r=L/s),this.set(i,r,s,t),this}let g=Math.sqrt((d-_)*(d-_)+(p-y)*(p-y)+(f-u)*(f-u));return Math.abs(g)<.001&&(g=1),this.x=(d-_)/g,this.y=(p-y)/g,this.z=(f-u)/g,this.w=Math.acos((c+m+h-1)/2),this}setFromMatrixPosition(e){let t=e.elements;return this.x=t[12],this.y=t[13],this.z=t[14],this.w=t[15],this}min(e){return this.x=Math.min(this.x,e.x),this.y=Math.min(this.y,e.y),this.z=Math.min(this.z,e.z),this.w=Math.min(this.w,e.w),this}max(e){return this.x=Math.max(this.x,e.x),this.y=Math.max(this.y,e.y),this.z=Math.max(this.z,e.z),this.w=Math.max(this.w,e.w),this}clamp(e,t){return this.x=Math.max(e.x,Math.min(t.x,this.x)),this.y=Math.max(e.y,Math.min(t.y,this.y)),this.z=Math.max(e.z,Math.min(t.z,this.z)),this.w=Math.max(e.w,Math.min(t.w,this.w)),this}clampScalar(e,t){return this.x=Math.max(e,Math.min(t,this.x)),this.y=Math.max(e,Math.min(t,this.y)),this.z=Math.max(e,Math.min(t,this.z)),this.w=Math.max(e,Math.min(t,this.w)),this}clampLength(e,t){let i=this.length();return this.divideScalar(i||1).multiplyScalar(Math.max(e,Math.min(t,i)))}floor(){return this.x=Math.floor(this.x),this.y=Math.floor(this.y),this.z=Math.floor(this.z),this.w=Math.floor(this.w),this}ceil(){return this.x=Math.ceil(this.x),this.y=Math.ceil(this.y),this.z=Math.ceil(this.z),this.w=Math.ceil(this.w),this}round(){return this.x=Math.round(this.x),this.y=Math.round(this.y),this.z=Math.round(this.z),this.w=Math.round(this.w),this}roundToZero(){return this.x=Math.trunc(this.x),this.y=Math.trunc(this.y),this.z=Math.trunc(this.z),this.w=Math.trunc(this.w),this}negate(){return this.x=-this.x,this.y=-this.y,this.z=-this.z,this.w=-this.w,this}dot(e){return this.x*e.x+this.y*e.y+this.z*e.z+this.w*e.w}lengthSq(){return this.x*this.x+this.y*this.y+this.z*this.z+this.w*this.w}length(){return Math.sqrt(this.x*this.x+this.y*this.y+this.z*this.z+this.w*this.w)}manhattanLength(){return Math.abs(this.x)+Math.abs(this.y)+Math.abs(this.z)+Math.abs(this.w)}normalize(){return this.divideScalar(this.length()||1)}setLength(e){return this.normalize().multiplyScalar(e)}lerp(e,t){return this.x+=(e.x-this.x)*t,this.y+=(e.y-this.y)*t,this.z+=(e.z-this.z)*t,this.w+=(e.w-this.w)*t,this}lerpVectors(e,t,i){return this.x=e.x+(t.x-e.x)*i,this.y=e.y+(t.y-e.y)*i,this.z=e.z+(t.z-e.z)*i,this.w=e.w+(t.w-e.w)*i,this}equals(e){return e.x===this.x&&e.y===this.y&&e.z===this.z&&e.w===this.w}fromArray(e,t=0){return this.x=e[t],this.y=e[t+1],this.z=e[t+2],this.w=e[t+3],this}toArray(e=[],t=0){return e[t]=this.x,e[t+1]=this.y,e[t+2]=this.z,e[t+3]=this.w,e}fromBufferAttribute(e,t){return this.x=e.getX(t),this.y=e.getY(t),this.z=e.getZ(t),this.w=e.getW(t),this}random(){return this.x=Math.random(),this.y=Math.random(),this.z=Math.random(),this.w=Math.random(),this}*[Symbol.iterator](){yield this.x,yield this.y,yield this.z,yield this.w}},Ff=class extends Zi{constructor(e=1,t=1,i={}){super(),this.isRenderTarget=!0,this.width=e,this.height=t,this.depth=1,this.scissor=new wt(0,0,e,t),this.scissorTest=!1,this.viewport=new wt(0,0,e,t);let r={width:e,height:t,depth:1};i=Object.assign({generateMipmaps:!1,internalFormat:null,minFilter:qt,depthBuffer:!0,stencilBuffer:!1,resolveDepthBuffer:!0,resolveStencilBuffer:!0,depthTexture:null,samples:0,count:1},i);let s=new $t(r,i.mapping,i.wrapS,i.wrapT,i.magFilter,i.minFilter,i.format,i.type,i.anisotropy,i.colorSpace);s.flipY=!1,s.generateMipmaps=i.generateMipmaps,s.internalFormat=i.internalFormat,this.textures=[];let o=i.count;for(let a=0;a<o;a++)this.textures[a]=s.clone(),this.textures[a].isRenderTargetTexture=!0;this.depthBuffer=i.depthBuffer,this.stencilBuffer=i.stencilBuffer,this.resolveDepthBuffer=i.resolveDepthBuffer,this.resolveStencilBuffer=i.resolveStencilBuffer,this.depthTexture=i.depthTexture,this.samples=i.samples}get texture(){return this.textures[0]}set texture(e){this.textures[0]=e}setSize(e,t,i=1){if(this.width!==e||this.height!==t||this.depth!==i){this.width=e,this.height=t,this.depth=i;for(let r=0,s=this.textures.length;r<s;r++)this.textures[r].image.width=e,this.textures[r].image.height=t,this.textures[r].image.depth=i;this.dispose()}this.viewport.set(0,0,e,t),this.scissor.set(0,0,e,t)}clone(){return new this.constructor().copy(this)}copy(e){this.width=e.width,this.height=e.height,this.depth=e.depth,this.scissor.copy(e.scissor),this.scissorTest=e.scissorTest,this.viewport.copy(e.viewport),this.textures.length=0;for(let i=0,r=e.textures.length;i<r;i++)this.textures[i]=e.textures[i].clone(),this.textures[i].isRenderTargetTexture=!0;let t=Object.assign({},e.texture.image);return this.texture.source=new dc(t),this.depthBuffer=e.depthBuffer,this.stencilBuffer=e.stencilBuffer,this.resolveDepthBuffer=e.resolveDepthBuffer,this.resolveStencilBuffer=e.resolveStencilBuffer,e.depthTexture!==null&&(this.depthTexture=e.depthTexture.clone()),this.samples=e.samples,this}dispose(){this.dispatchEvent({type:"dispose"})}},fi=class extends Ff{constructor(e=1,t=1,i={}){super(e,t,i),this.isWebGLRenderTarget=!0}},fc=class extends $t{constructor(e=null,t=1,i=1,r=1){super(null),this.isDataArrayTexture=!0,this.image={data:e,width:t,height:i,depth:r},this.magFilter=Sn,this.minFilter=Sn,this.wrapR=Er,this.generateMipmaps=!1,this.flipY=!1,this.unpackAlignment=1,this.layerUpdates=new Set}addLayerUpdate(e){this.layerUpdates.add(e)}clearLayerUpdates(){this.layerUpdates.clear()}};var Bf=class extends $t{constructor(e=null,t=1,i=1,r=1){super(null),this.isData3DTexture=!0,this.image={data:e,width:t,height:i,depth:r},this.magFilter=Sn,this.minFilter=Sn,this.wrapR=Er,this.generateMipmaps=!1,this.flipY=!1,this.unpackAlignment=1}};var Nn=class{constructor(e=0,t=0,i=0,r=1){this.isQuaternion=!0,this._x=e,this._y=t,this._z=i,this._w=r}static slerpFlat(e,t,i,r,s,o,a){let l=i[r+0],c=i[r+1],u=i[r+2],p=i[r+3],f=s[o+0],m=s[o+1],_=s[o+2],y=s[o+3];if(a===0){e[t+0]=l,e[t+1]=c,e[t+2]=u,e[t+3]=p;return}if(a===1){e[t+0]=f,e[t+1]=m,e[t+2]=_,e[t+3]=y;return}if(p!==y||l!==f||c!==m||u!==_){let d=1-a,h=l*f+c*m+u*_+p*y,g=h>=0?1:-1,v=1-h*h;if(v>Number.EPSILON){let R=Math.sqrt(v),C=Math.atan2(R,h*g);d=Math.sin(d*C)/R,a=Math.sin(a*C)/R}let x=a*g;if(l=l*d+f*x,c=c*d+m*x,u=u*d+_*x,p=p*d+y*x,d===1-a){let R=1/Math.sqrt(l*l+c*c+u*u+p*p);l*=R,c*=R,u*=R,p*=R}}e[t]=l,e[t+1]=c,e[t+2]=u,e[t+3]=p}static multiplyQuaternionsFlat(e,t,i,r,s,o){let a=i[r],l=i[r+1],c=i[r+2],u=i[r+3],p=s[o],f=s[o+1],m=s[o+2],_=s[o+3];return e[t]=a*_+u*p+l*m-c*f,e[t+1]=l*_+u*f+c*p-a*m,e[t+2]=c*_+u*m+a*f-l*p,e[t+3]=u*_-a*p-l*f-c*m,e}get x(){return this._x}set x(e){this._x=e,this._onChangeCallback()}get y(){return this._y}set y(e){this._y=e,this._onChangeCallback()}get z(){return this._z}set z(e){this._z=e,this._onChangeCallback()}get w(){return this._w}set w(e){this._w=e,this._onChangeCallback()}set(e,t,i,r){return this._x=e,this._y=t,this._z=i,this._w=r,this._onChangeCallback(),this}clone(){return new this.constructor(this._x,this._y,this._z,this._w)}copy(e){return this._x=e.x,this._y=e.y,this._z=e.z,this._w=e.w,this._onChangeCallback(),this}setFromEuler(e,t=!0){let i=e._x,r=e._y,s=e._z,o=e._order,a=Math.cos,l=Math.sin,c=a(i/2),u=a(r/2),p=a(s/2),f=l(i/2),m=l(r/2),_=l(s/2);switch(o){case"XYZ":this._x=f*u*p+c*m*_,this._y=c*m*p-f*u*_,this._z=c*u*_+f*m*p,this._w=c*u*p-f*m*_;break;case"YXZ":this._x=f*u*p+c*m*_,this._y=c*m*p-f*u*_,this._z=c*u*_-f*m*p,this._w=c*u*p+f*m*_;break;case"ZXY":this._x=f*u*p-c*m*_,this._y=c*m*p+f*u*_,this._z=c*u*_+f*m*p,this._w=c*u*p-f*m*_;break;case"ZYX":this._x=f*u*p-c*m*_,this._y=c*m*p+f*u*_,this._z=c*u*_-f*m*p,this._w=c*u*p+f*m*_;break;case"YZX":this._x=f*u*p+c*m*_,this._y=c*m*p+f*u*_,this._z=c*u*_-f*m*p,this._w=c*u*p-f*m*_;break;case"XZY":this._x=f*u*p-c*m*_,this._y=c*m*p-f*u*_,this._z=c*u*_+f*m*p,this._w=c*u*p+f*m*_;break;default:console.warn("THREE.Quaternion: .setFromEuler() encountered an unknown order: "+o)}return t===!0&&this._onChangeCallback(),this}setFromAxisAngle(e,t){let i=t/2,r=Math.sin(i);return this._x=e.x*r,this._y=e.y*r,this._z=e.z*r,this._w=Math.cos(i),this._onChangeCallback(),this}setFromRotationMatrix(e){let t=e.elements,i=t[0],r=t[4],s=t[8],o=t[1],a=t[5],l=t[9],c=t[2],u=t[6],p=t[10],f=i+a+p;if(f>0){let m=.5/Math.sqrt(f+1);this._w=.25/m,this._x=(u-l)*m,this._y=(s-c)*m,this._z=(o-r)*m}else if(i>a&&i>p){let m=2*Math.sqrt(1+i-a-p);this._w=(u-l)/m,this._x=.25*m,this._y=(r+o)/m,this._z=(s+c)/m}else if(a>p){let m=2*Math.sqrt(1+a-i-p);this._w=(s-c)/m,this._x=(r+o)/m,this._y=.25*m,this._z=(l+u)/m}else{let m=2*Math.sqrt(1+p-i-a);this._w=(o-r)/m,this._x=(s+c)/m,this._y=(l+u)/m,this._z=.25*m}return this._onChangeCallback(),this}setFromUnitVectors(e,t){let i=e.dot(t)+1;return i<Number.EPSILON?(i=0,Math.abs(e.x)>Math.abs(e.z)?(this._x=-e.y,this._y=e.x,this._z=0,this._w=i):(this._x=0,this._y=-e.z,this._z=e.y,this._w=i)):(this._x=e.y*t.z-e.z*t.y,this._y=e.z*t.x-e.x*t.z,this._z=e.x*t.y-e.y*t.x,this._w=i),this.normalize()}angleTo(e){return 2*Math.acos(Math.abs(en(this.dot(e),-1,1)))}rotateTowards(e,t){let i=this.angleTo(e);if(i===0)return this;let r=Math.min(1,t/i);return this.slerp(e,r),this}identity(){return this.set(0,0,0,1)}invert(){return this.conjugate()}conjugate(){return this._x*=-1,this._y*=-1,this._z*=-1,this._onChangeCallback(),this}dot(e){return this._x*e._x+this._y*e._y+this._z*e._z+this._w*e._w}lengthSq(){return this._x*this._x+this._y*this._y+this._z*this._z+this._w*this._w}length(){return Math.sqrt(this._x*this._x+this._y*this._y+this._z*this._z+this._w*this._w)}normalize(){let e=this.length();return e===0?(this._x=0,this._y=0,this._z=0,this._w=1):(e=1/e,this._x=this._x*e,this._y=this._y*e,this._z=this._z*e,this._w=this._w*e),this._onChangeCallback(),this}multiply(e){return this.multiplyQuaternions(this,e)}premultiply(e){return this.multiplyQuaternions(e,this)}multiplyQuaternions(e,t){let i=e._x,r=e._y,s=e._z,o=e._w,a=t._x,l=t._y,c=t._z,u=t._w;return this._x=i*u+o*a+r*c-s*l,this._y=r*u+o*l+s*a-i*c,this._z=s*u+o*c+i*l-r*a,this._w=o*u-i*a-r*l-s*c,this._onChangeCallback(),this}slerp(e,t){if(t===0)return this;if(t===1)return this.copy(e);let i=this._x,r=this._y,s=this._z,o=this._w,a=o*e._w+i*e._x+r*e._y+s*e._z;if(a<0?(this._w=-e._w,this._x=-e._x,this._y=-e._y,this._z=-e._z,a=-a):this.copy(e),a>=1)return this._w=o,this._x=i,this._y=r,this._z=s,this;let l=1-a*a;if(l<=Number.EPSILON){let m=1-t;return this._w=m*o+t*this._w,this._x=m*i+t*this._x,this._y=m*r+t*this._y,this._z=m*s+t*this._z,this.normalize(),this}let c=Math.sqrt(l),u=Math.atan2(c,a),p=Math.sin((1-t)*u)/c,f=Math.sin(t*u)/c;return this._w=o*p+this._w*f,this._x=i*p+this._x*f,this._y=r*p+this._y*f,this._z=s*p+this._z*f,this._onChangeCallback(),this}slerpQuaternions(e,t,i){return this.copy(e).slerp(t,i)}random(){let e=2*Math.PI*Math.random(),t=2*Math.PI*Math.random(),i=Math.random(),r=Math.sqrt(1-i),s=Math.sqrt(i);return this.set(r*Math.sin(e),r*Math.cos(e),s*Math.sin(t),s*Math.cos(t))}equals(e){return e._x===this._x&&e._y===this._y&&e._z===this._z&&e._w===this._w}fromArray(e,t=0){return this._x=e[t],this._y=e[t+1],this._z=e[t+2],this._w=e[t+3],this._onChangeCallback(),this}toArray(e=[],t=0){return e[t]=this._x,e[t+1]=this._y,e[t+2]=this._z,e[t+3]=this._w,e}fromBufferAttribute(e,t){return this._x=e.getX(t),this._y=e.getY(t),this._z=e.getZ(t),this._w=e.getW(t),this._onChangeCallback(),this}toJSON(){return this.toArray()}_onChange(e){return this._onChangeCallback=e,this}_onChangeCallback(){}*[Symbol.iterator](){yield this._x,yield this._y,yield this._z,yield this._w}},D=class n{constructor(e=0,t=0,i=0){n.prototype.isVector3=!0,this.x=e,this.y=t,this.z=i}set(e,t,i){return i===void 0&&(i=this.z),this.x=e,this.y=t,this.z=i,this}setScalar(e){return this.x=e,this.y=e,this.z=e,this}setX(e){return this.x=e,this}setY(e){return this.y=e,this}setZ(e){return this.z=e,this}setComponent(e,t){switch(e){case 0:this.x=t;break;case 1:this.y=t;break;case 2:this.z=t;break;default:throw new Error("index is out of range: "+e)}return this}getComponent(e){switch(e){case 0:return this.x;case 1:return this.y;case 2:return this.z;default:throw new Error("index is out of range: "+e)}}clone(){return new this.constructor(this.x,this.y,this.z)}copy(e){return this.x=e.x,this.y=e.y,this.z=e.z,this}add(e){return this.x+=e.x,this.y+=e.y,this.z+=e.z,this}addScalar(e){return this.x+=e,this.y+=e,this.z+=e,this}addVectors(e,t){return this.x=e.x+t.x,this.y=e.y+t.y,this.z=e.z+t.z,this}addScaledVector(e,t){return this.x+=e.x*t,this.y+=e.y*t,this.z+=e.z*t,this}sub(e){return this.x-=e.x,this.y-=e.y,this.z-=e.z,this}subScalar(e){return this.x-=e,this.y-=e,this.z-=e,this}subVectors(e,t){return this.x=e.x-t.x,this.y=e.y-t.y,this.z=e.z-t.z,this}multiply(e){return this.x*=e.x,this.y*=e.y,this.z*=e.z,this}multiplyScalar(e){return this.x*=e,this.y*=e,this.z*=e,this}multiplyVectors(e,t){return this.x=e.x*t.x,this.y=e.y*t.y,this.z=e.z*t.z,this}applyEuler(e){return this.applyQuaternion(A_.setFromEuler(e))}applyAxisAngle(e,t){return this.applyQuaternion(A_.setFromAxisAngle(e,t))}applyMatrix3(e){let t=this.x,i=this.y,r=this.z,s=e.elements;return this.x=s[0]*t+s[3]*i+s[6]*r,this.y=s[1]*t+s[4]*i+s[7]*r,this.z=s[2]*t+s[5]*i+s[8]*r,this}applyNormalMatrix(e){return this.applyMatrix3(e).normalize()}applyMatrix4(e){let t=this.x,i=this.y,r=this.z,s=e.elements,o=1/(s[3]*t+s[7]*i+s[11]*r+s[15]);return this.x=(s[0]*t+s[4]*i+s[8]*r+s[12])*o,this.y=(s[1]*t+s[5]*i+s[9]*r+s[13])*o,this.z=(s[2]*t+s[6]*i+s[10]*r+s[14])*o,this}applyQuaternion(e){let t=this.x,i=this.y,r=this.z,s=e.x,o=e.y,a=e.z,l=e.w,c=2*(o*r-a*i),u=2*(a*t-s*r),p=2*(s*i-o*t);return this.x=t+l*c+o*p-a*u,this.y=i+l*u+a*c-s*p,this.z=r+l*p+s*u-o*c,this}project(e){return this.applyMatrix4(e.matrixWorldInverse).applyMatrix4(e.projectionMatrix)}unproject(e){return this.applyMatrix4(e.projectionMatrixInverse).applyMatrix4(e.matrixWorld)}transformDirection(e){let t=this.x,i=this.y,r=this.z,s=e.elements;return this.x=s[0]*t+s[4]*i+s[8]*r,this.y=s[1]*t+s[5]*i+s[9]*r,this.z=s[2]*t+s[6]*i+s[10]*r,this.normalize()}divide(e){return this.x/=e.x,this.y/=e.y,this.z/=e.z,this}divideScalar(e){return this.multiplyScalar(1/e)}min(e){return this.x=Math.min(this.x,e.x),this.y=Math.min(this.y,e.y),this.z=Math.min(this.z,e.z),this}max(e){return this.x=Math.max(this.x,e.x),this.y=Math.max(this.y,e.y),this.z=Math.max(this.z,e.z),this}clamp(e,t){return this.x=Math.max(e.x,Math.min(t.x,this.x)),this.y=Math.max(e.y,Math.min(t.y,this.y)),this.z=Math.max(e.z,Math.min(t.z,this.z)),this}clampScalar(e,t){return this.x=Math.max(e,Math.min(t,this.x)),this.y=Math.max(e,Math.min(t,this.y)),this.z=Math.max(e,Math.min(t,this.z)),this}clampLength(e,t){let i=this.length();return this.divideScalar(i||1).multiplyScalar(Math.max(e,Math.min(t,i)))}floor(){return this.x=Math.floor(this.x),this.y=Math.floor(this.y),this.z=Math.floor(this.z),this}ceil(){return this.x=Math.ceil(this.x),this.y=Math.ceil(this.y),this.z=Math.ceil(this.z),this}round(){return this.x=Math.round(this.x),this.y=Math.round(this.y),this.z=Math.round(this.z),this}roundToZero(){return this.x=Math.trunc(this.x),this.y=Math.trunc(this.y),this.z=Math.trunc(this.z),this}negate(){return this.x=-this.x,this.y=-this.y,this.z=-this.z,this}dot(e){return this.x*e.x+this.y*e.y+this.z*e.z}lengthSq(){return this.x*this.x+this.y*this.y+this.z*this.z}length(){return Math.sqrt(this.x*this.x+this.y*this.y+this.z*this.z)}manhattanLength(){return Math.abs(this.x)+Math.abs(this.y)+Math.abs(this.z)}normalize(){return this.divideScalar(this.length()||1)}setLength(e){return this.normalize().multiplyScalar(e)}lerp(e,t){return this.x+=(e.x-this.x)*t,this.y+=(e.y-this.y)*t,this.z+=(e.z-this.z)*t,this}lerpVectors(e,t,i){return this.x=e.x+(t.x-e.x)*i,this.y=e.y+(t.y-e.y)*i,this.z=e.z+(t.z-e.z)*i,this}cross(e){return this.crossVectors(this,e)}crossVectors(e,t){let i=e.x,r=e.y,s=e.z,o=t.x,a=t.y,l=t.z;return this.x=r*l-s*a,this.y=s*o-i*l,this.z=i*a-r*o,this}projectOnVector(e){let t=e.lengthSq();if(t===0)return this.set(0,0,0);let i=e.dot(this)/t;return this.copy(e).multiplyScalar(i)}projectOnPlane(e){return Ud.copy(this).projectOnVector(e),this.sub(Ud)}reflect(e){return this.sub(Ud.copy(e).multiplyScalar(2*this.dot(e)))}angleTo(e){let t=Math.sqrt(this.lengthSq()*e.lengthSq());if(t===0)return Math.PI/2;let i=this.dot(e)/t;return Math.acos(en(i,-1,1))}distanceTo(e){return Math.sqrt(this.distanceToSquared(e))}distanceToSquared(e){let t=this.x-e.x,i=this.y-e.y,r=this.z-e.z;return t*t+i*i+r*r}manhattanDistanceTo(e){return Math.abs(this.x-e.x)+Math.abs(this.y-e.y)+Math.abs(this.z-e.z)}setFromSpherical(e){return this.setFromSphericalCoords(e.radius,e.phi,e.theta)}setFromSphericalCoords(e,t,i){let r=Math.sin(t)*e;return this.x=r*Math.sin(i),this.y=Math.cos(t)*e,this.z=r*Math.cos(i),this}setFromCylindrical(e){return this.setFromCylindricalCoords(e.radius,e.theta,e.y)}setFromCylindricalCoords(e,t,i){return this.x=e*Math.sin(t),this.y=i,this.z=e*Math.cos(t),this}setFromMatrixPosition(e){let t=e.elements;return this.x=t[12],this.y=t[13],this.z=t[14],this}setFromMatrixScale(e){let t=this.setFromMatrixColumn(e,0).length(),i=this.setFromMatrixColumn(e,1).length(),r=this.setFromMatrixColumn(e,2).length();return this.x=t,this.y=i,this.z=r,this}setFromMatrixColumn(e,t){return this.fromArray(e.elements,t*4)}setFromMatrix3Column(e,t){return this.fromArray(e.elements,t*3)}setFromEuler(e){return this.x=e._x,this.y=e._y,this.z=e._z,this}setFromColor(e){return this.x=e.r,this.y=e.g,this.z=e.b,this}equals(e){return e.x===this.x&&e.y===this.y&&e.z===this.z}fromArray(e,t=0){return this.x=e[t],this.y=e[t+1],this.z=e[t+2],this}toArray(e=[],t=0){return e[t]=this.x,e[t+1]=this.y,e[t+2]=this.z,e}fromBufferAttribute(e,t){return this.x=e.getX(t),this.y=e.getY(t),this.z=e.getZ(t),this}random(){return this.x=Math.random(),this.y=Math.random(),this.z=Math.random(),this}randomDirection(){let e=Math.random()*Math.PI*2,t=Math.random()*2-1,i=Math.sqrt(1-t*t);return this.x=i*Math.cos(e),this.y=t,this.z=i*Math.sin(e),this}*[Symbol.iterator](){yield this.x,yield this.y,yield this.z}},Ud=new D,A_=new Nn,br=class{constructor(e=new D(1/0,1/0,1/0),t=new D(-1/0,-1/0,-1/0)){this.isBox3=!0,this.min=e,this.max=t}set(e,t){return this.min.copy(e),this.max.copy(t),this}setFromArray(e){this.makeEmpty();for(let t=0,i=e.length;t<i;t+=3)this.expandByPoint(Rn.fromArray(e,t));return this}setFromBufferAttribute(e){this.makeEmpty();for(let t=0,i=e.count;t<i;t++)this.expandByPoint(Rn.fromBufferAttribute(e,t));return this}setFromPoints(e){this.makeEmpty();for(let t=0,i=e.length;t<i;t++)this.expandByPoint(e[t]);return this}setFromCenterAndSize(e,t){let i=Rn.copy(t).multiplyScalar(.5);return this.min.copy(e).sub(i),this.max.copy(e).add(i),this}setFromObject(e,t=!1){return this.makeEmpty(),this.expandByObject(e,t)}clone(){return new this.constructor().copy(this)}copy(e){return this.min.copy(e.min),this.max.copy(e.max),this}makeEmpty(){return this.min.x=this.min.y=this.min.z=1/0,this.max.x=this.max.y=this.max.z=-1/0,this}isEmpty(){return this.max.x<this.min.x||this.max.y<this.min.y||this.max.z<this.min.z}getCenter(e){return this.isEmpty()?e.set(0,0,0):e.addVectors(this.min,this.max).multiplyScalar(.5)}getSize(e){return this.isEmpty()?e.set(0,0,0):e.subVectors(this.max,this.min)}expandByPoint(e){return this.min.min(e),this.max.max(e),this}expandByVector(e){return this.min.sub(e),this.max.add(e),this}expandByScalar(e){return this.min.addScalar(-e),this.max.addScalar(e),this}expandByObject(e,t=!1){e.updateWorldMatrix(!1,!1);let i=e.geometry;if(i!==void 0){let s=i.getAttribute("position");if(t===!0&&s!==void 0&&e.isInstancedMesh!==!0)for(let o=0,a=s.count;o<a;o++)e.isMesh===!0?e.getVertexPosition(o,Rn):Rn.fromBufferAttribute(s,o),Rn.applyMatrix4(e.matrixWorld),this.expandByPoint(Rn);else e.boundingBox!==void 0?(e.boundingBox===null&&e.computeBoundingBox(),Nl.copy(e.boundingBox)):(i.boundingBox===null&&i.computeBoundingBox(),Nl.copy(i.boundingBox)),Nl.applyMatrix4(e.matrixWorld),this.union(Nl)}let r=e.children;for(let s=0,o=r.length;s<o;s++)this.expandByObject(r[s],t);return this}containsPoint(e){return!(e.x<this.min.x||e.x>this.max.x||e.y<this.min.y||e.y>this.max.y||e.z<this.min.z||e.z>this.max.z)}containsBox(e){return this.min.x<=e.min.x&&e.max.x<=this.max.x&&this.min.y<=e.min.y&&e.max.y<=this.max.y&&this.min.z<=e.min.z&&e.max.z<=this.max.z}getParameter(e,t){return t.set((e.x-this.min.x)/(this.max.x-this.min.x),(e.y-this.min.y)/(this.max.y-this.min.y),(e.z-this.min.z)/(this.max.z-this.min.z))}intersectsBox(e){return!(e.max.x<this.min.x||e.min.x>this.max.x||e.max.y<this.min.y||e.min.y>this.max.y||e.max.z<this.min.z||e.min.z>this.max.z)}intersectsSphere(e){return this.clampPoint(e.center,Rn),Rn.distanceToSquared(e.center)<=e.radius*e.radius}intersectsPlane(e){let t,i;return e.normal.x>0?(t=e.normal.x*this.min.x,i=e.normal.x*this.max.x):(t=e.normal.x*this.max.x,i=e.normal.x*this.min.x),e.normal.y>0?(t+=e.normal.y*this.min.y,i+=e.normal.y*this.max.y):(t+=e.normal.y*this.max.y,i+=e.normal.y*this.min.y),e.normal.z>0?(t+=e.normal.z*this.min.z,i+=e.normal.z*this.max.z):(t+=e.normal.z*this.max.z,i+=e.normal.z*this.min.z),t<=-e.constant&&i>=-e.constant}intersectsTriangle(e){if(this.isEmpty())return!1;this.getCenter(Zo),Dl.subVectors(this.max,Zo),gs.subVectors(e.a,Zo),_s.subVectors(e.b,Zo),vs.subVectors(e.c,Zo),Bi.subVectors(_s,gs),ki.subVectors(vs,_s),mr.subVectors(gs,vs);let t=[0,-Bi.z,Bi.y,0,-ki.z,ki.y,0,-mr.z,mr.y,Bi.z,0,-Bi.x,ki.z,0,-ki.x,mr.z,0,-mr.x,-Bi.y,Bi.x,0,-ki.y,ki.x,0,-mr.y,mr.x,0];return!Nd(t,gs,_s,vs,Dl)||(t=[1,0,0,0,1,0,0,0,1],!Nd(t,gs,_s,vs,Dl))?!1:(Ol.crossVectors(Bi,ki),t=[Ol.x,Ol.y,Ol.z],Nd(t,gs,_s,vs,Dl))}clampPoint(e,t){return t.copy(e).clamp(this.min,this.max)}distanceToPoint(e){return this.clampPoint(e,Rn).distanceTo(e)}getBoundingSphere(e){return this.isEmpty()?e.makeEmpty():(this.getCenter(e.center),e.radius=this.getSize(Rn).length()*.5),e}intersect(e){return this.min.max(e.min),this.max.min(e.max),this.isEmpty()&&this.makeEmpty(),this}union(e){return this.min.min(e.min),this.max.max(e.max),this}applyMatrix4(e){return this.isEmpty()?this:(ti[0].set(this.min.x,this.min.y,this.min.z).applyMatrix4(e),ti[1].set(this.min.x,this.min.y,this.max.z).applyMatrix4(e),ti[2].set(this.min.x,this.max.y,this.min.z).applyMatrix4(e),ti[3].set(this.min.x,this.max.y,this.max.z).applyMatrix4(e),ti[4].set(this.max.x,this.min.y,this.min.z).applyMatrix4(e),ti[5].set(this.max.x,this.min.y,this.max.z).applyMatrix4(e),ti[6].set(this.max.x,this.max.y,this.min.z).applyMatrix4(e),ti[7].set(this.max.x,this.max.y,this.max.z).applyMatrix4(e),this.setFromPoints(ti),this)}translate(e){return this.min.add(e),this.max.add(e),this}equals(e){return e.min.equals(this.min)&&e.max.equals(this.max)}},ti=[new D,new D,new D,new D,new D,new D,new D,new D],Rn=new D,Nl=new br,gs=new D,_s=new D,vs=new D,Bi=new D,ki=new D,mr=new D,Zo=new D,Dl=new D,Ol=new D,gr=new D;function Nd(n,e,t,i,r){for(let s=0,o=n.length-3;s<=o;s+=3){gr.fromArray(n,s);let a=r.x*Math.abs(gr.x)+r.y*Math.abs(gr.y)+r.z*Math.abs(gr.z),l=e.dot(gr),c=t.dot(gr),u=i.dot(gr);if(Math.max(-Math.max(l,c,u),Math.min(l,c,u))>a)return!1}return!0}var PS=new br,Jo=new D,Dd=new D,ia=class{constructor(e=new D,t=-1){this.isSphere=!0,this.center=e,this.radius=t}set(e,t){return this.center.copy(e),this.radius=t,this}setFromPoints(e,t){let i=this.center;t!==void 0?i.copy(t):PS.setFromPoints(e).getCenter(i);let r=0;for(let s=0,o=e.length;s<o;s++)r=Math.max(r,i.distanceToSquared(e[s]));return this.radius=Math.sqrt(r),this}copy(e){return this.center.copy(e.center),this.radius=e.radius,this}isEmpty(){return this.radius<0}makeEmpty(){return this.center.set(0,0,0),this.radius=-1,this}containsPoint(e){return e.distanceToSquared(this.center)<=this.radius*this.radius}distanceToPoint(e){return e.distanceTo(this.center)-this.radius}intersectsSphere(e){let t=this.radius+e.radius;return e.center.distanceToSquared(this.center)<=t*t}intersectsBox(e){return e.intersectsSphere(this)}intersectsPlane(e){return Math.abs(e.distanceToPoint(this.center))<=this.radius}clampPoint(e,t){let i=this.center.distanceToSquared(e);return t.copy(e),i>this.radius*this.radius&&(t.sub(this.center).normalize(),t.multiplyScalar(this.radius).add(this.center)),t}getBoundingBox(e){return this.isEmpty()?(e.makeEmpty(),e):(e.set(this.center,this.center),e.expandByScalar(this.radius),e)}applyMatrix4(e){return this.center.applyMatrix4(e),this.radius=this.radius*e.getMaxScaleOnAxis(),this}translate(e){return this.center.add(e),this}expandByPoint(e){if(this.isEmpty())return this.center.copy(e),this.radius=0,this;Jo.subVectors(e,this.center);let t=Jo.lengthSq();if(t>this.radius*this.radius){let i=Math.sqrt(t),r=(i-this.radius)*.5;this.center.addScaledVector(Jo,r/i),this.radius+=r}return this}union(e){return e.isEmpty()?this:this.isEmpty()?(this.copy(e),this):(this.center.equals(e.center)===!0?this.radius=Math.max(this.radius,e.radius):(Dd.subVectors(e.center,this.center).setLength(e.radius),this.expandByPoint(Jo.copy(e.center).add(Dd)),this.expandByPoint(Jo.copy(e.center).sub(Dd))),this)}equals(e){return e.center.equals(this.center)&&e.radius===this.radius}clone(){return new this.constructor().copy(this)}},ni=new D,Od=new D,Fl=new D,zi=new D,Fd=new D,Bl=new D,Bd=new D,kf=class{constructor(e=new D,t=new D(0,0,-1)){this.origin=e,this.direction=t}set(e,t){return this.origin.copy(e),this.direction.copy(t),this}copy(e){return this.origin.copy(e.origin),this.direction.copy(e.direction),this}at(e,t){return t.copy(this.origin).addScaledVector(this.direction,e)}lookAt(e){return this.direction.copy(e).sub(this.origin).normalize(),this}recast(e){return this.origin.copy(this.at(e,ni)),this}closestPointToPoint(e,t){t.subVectors(e,this.origin);let i=t.dot(this.direction);return i<0?t.copy(this.origin):t.copy(this.origin).addScaledVector(this.direction,i)}distanceToPoint(e){return Math.sqrt(this.distanceSqToPoint(e))}distanceSqToPoint(e){let t=ni.subVectors(e,this.origin).dot(this.direction);return t<0?this.origin.distanceToSquared(e):(ni.copy(this.origin).addScaledVector(this.direction,t),ni.distanceToSquared(e))}distanceSqToSegment(e,t,i,r){Od.copy(e).add(t).multiplyScalar(.5),Fl.copy(t).sub(e).normalize(),zi.copy(this.origin).sub(Od);let s=e.distanceTo(t)*.5,o=-this.direction.dot(Fl),a=zi.dot(this.direction),l=-zi.dot(Fl),c=zi.lengthSq(),u=Math.abs(1-o*o),p,f,m,_;if(u>0)if(p=o*l-a,f=o*a-l,_=s*u,p>=0)if(f>=-_)if(f<=_){let y=1/u;p*=y,f*=y,m=p*(p+o*f+2*a)+f*(o*p+f+2*l)+c}else f=s,p=Math.max(0,-(o*f+a)),m=-p*p+f*(f+2*l)+c;else f=-s,p=Math.max(0,-(o*f+a)),m=-p*p+f*(f+2*l)+c;else f<=-_?(p=Math.max(0,-(-o*s+a)),f=p>0?-s:Math.min(Math.max(-s,-l),s),m=-p*p+f*(f+2*l)+c):f<=_?(p=0,f=Math.min(Math.max(-s,-l),s),m=f*(f+2*l)+c):(p=Math.max(0,-(o*s+a)),f=p>0?s:Math.min(Math.max(-s,-l),s),m=-p*p+f*(f+2*l)+c);else f=o>0?-s:s,p=Math.max(0,-(o*f+a)),m=-p*p+f*(f+2*l)+c;return i&&i.copy(this.origin).addScaledVector(this.direction,p),r&&r.copy(Od).addScaledVector(Fl,f),m}intersectSphere(e,t){ni.subVectors(e.center,this.origin);let i=ni.dot(this.direction),r=ni.dot(ni)-i*i,s=e.radius*e.radius;if(r>s)return null;let o=Math.sqrt(s-r),a=i-o,l=i+o;return l<0?null:a<0?this.at(l,t):this.at(a,t)}intersectsSphere(e){return this.distanceSqToPoint(e.center)<=e.radius*e.radius}distanceToPlane(e){let t=e.normal.dot(this.direction);if(t===0)return e.distanceToPoint(this.origin)===0?0:null;let i=-(this.origin.dot(e.normal)+e.constant)/t;return i>=0?i:null}intersectPlane(e,t){let i=this.distanceToPlane(e);return i===null?null:this.at(i,t)}intersectsPlane(e){let t=e.distanceToPoint(this.origin);return t===0||e.normal.dot(this.direction)*t<0}intersectBox(e,t){let i,r,s,o,a,l,c=1/this.direction.x,u=1/this.direction.y,p=1/this.direction.z,f=this.origin;return c>=0?(i=(e.min.x-f.x)*c,r=(e.max.x-f.x)*c):(i=(e.max.x-f.x)*c,r=(e.min.x-f.x)*c),u>=0?(s=(e.min.y-f.y)*u,o=(e.max.y-f.y)*u):(s=(e.max.y-f.y)*u,o=(e.min.y-f.y)*u),i>o||s>r||((s>i||isNaN(i))&&(i=s),(o<r||isNaN(r))&&(r=o),p>=0?(a=(e.min.z-f.z)*p,l=(e.max.z-f.z)*p):(a=(e.max.z-f.z)*p,l=(e.min.z-f.z)*p),i>l||a>r)||((a>i||i!==i)&&(i=a),(l<r||r!==r)&&(r=l),r<0)?null:this.at(i>=0?i:r,t)}intersectsBox(e){return this.intersectBox(e,ni)!==null}intersectTriangle(e,t,i,r,s){Fd.subVectors(t,e),Bl.subVectors(i,e),Bd.crossVectors(Fd,Bl);let o=this.direction.dot(Bd),a;if(o>0){if(r)return null;a=1}else if(o<0)a=-1,o=-o;else return null;zi.subVectors(this.origin,e);let l=a*this.direction.dot(Bl.crossVectors(zi,Bl));if(l<0)return null;let c=a*this.direction.dot(Fd.cross(zi));if(c<0||l+c>o)return null;let u=-a*zi.dot(Bd);return u<0?null:this.at(u/o,s)}applyMatrix4(e){return this.origin.applyMatrix4(e),this.direction.transformDirection(e),this}equals(e){return e.origin.equals(this.origin)&&e.direction.equals(this.direction)}clone(){return new this.constructor().copy(this)}},et=class n{constructor(e,t,i,r,s,o,a,l,c,u,p,f,m,_,y,d){n.prototype.isMatrix4=!0,this.elements=[1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1],e!==void 0&&this.set(e,t,i,r,s,o,a,l,c,u,p,f,m,_,y,d)}set(e,t,i,r,s,o,a,l,c,u,p,f,m,_,y,d){let h=this.elements;return h[0]=e,h[4]=t,h[8]=i,h[12]=r,h[1]=s,h[5]=o,h[9]=a,h[13]=l,h[2]=c,h[6]=u,h[10]=p,h[14]=f,h[3]=m,h[7]=_,h[11]=y,h[15]=d,this}identity(){return this.set(1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1),this}clone(){return new n().fromArray(this.elements)}copy(e){let t=this.elements,i=e.elements;return t[0]=i[0],t[1]=i[1],t[2]=i[2],t[3]=i[3],t[4]=i[4],t[5]=i[5],t[6]=i[6],t[7]=i[7],t[8]=i[8],t[9]=i[9],t[10]=i[10],t[11]=i[11],t[12]=i[12],t[13]=i[13],t[14]=i[14],t[15]=i[15],this}copyPosition(e){let t=this.elements,i=e.elements;return t[12]=i[12],t[13]=i[13],t[14]=i[14],this}setFromMatrix3(e){let t=e.elements;return this.set(t[0],t[3],t[6],0,t[1],t[4],t[7],0,t[2],t[5],t[8],0,0,0,0,1),this}extractBasis(e,t,i){return e.setFromMatrixColumn(this,0),t.setFromMatrixColumn(this,1),i.setFromMatrixColumn(this,2),this}makeBasis(e,t,i){return this.set(e.x,t.x,i.x,0,e.y,t.y,i.y,0,e.z,t.z,i.z,0,0,0,0,1),this}extractRotation(e){let t=this.elements,i=e.elements,r=1/ys.setFromMatrixColumn(e,0).length(),s=1/ys.setFromMatrixColumn(e,1).length(),o=1/ys.setFromMatrixColumn(e,2).length();return t[0]=i[0]*r,t[1]=i[1]*r,t[2]=i[2]*r,t[3]=0,t[4]=i[4]*s,t[5]=i[5]*s,t[6]=i[6]*s,t[7]=0,t[8]=i[8]*o,t[9]=i[9]*o,t[10]=i[10]*o,t[11]=0,t[12]=0,t[13]=0,t[14]=0,t[15]=1,this}makeRotationFromEuler(e){let t=this.elements,i=e.x,r=e.y,s=e.z,o=Math.cos(i),a=Math.sin(i),l=Math.cos(r),c=Math.sin(r),u=Math.cos(s),p=Math.sin(s);if(e.order==="XYZ"){let f=o*u,m=o*p,_=a*u,y=a*p;t[0]=l*u,t[4]=-l*p,t[8]=c,t[1]=m+_*c,t[5]=f-y*c,t[9]=-a*l,t[2]=y-f*c,t[6]=_+m*c,t[10]=o*l}else if(e.order==="YXZ"){let f=l*u,m=l*p,_=c*u,y=c*p;t[0]=f+y*a,t[4]=_*a-m,t[8]=o*c,t[1]=o*p,t[5]=o*u,t[9]=-a,t[2]=m*a-_,t[6]=y+f*a,t[10]=o*l}else if(e.order==="ZXY"){let f=l*u,m=l*p,_=c*u,y=c*p;t[0]=f-y*a,t[4]=-o*p,t[8]=_+m*a,t[1]=m+_*a,t[5]=o*u,t[9]=y-f*a,t[2]=-o*c,t[6]=a,t[10]=o*l}else if(e.order==="ZYX"){let f=o*u,m=o*p,_=a*u,y=a*p;t[0]=l*u,t[4]=_*c-m,t[8]=f*c+y,t[1]=l*p,t[5]=y*c+f,t[9]=m*c-_,t[2]=-c,t[6]=a*l,t[10]=o*l}else if(e.order==="YZX"){let f=o*l,m=o*c,_=a*l,y=a*c;t[0]=l*u,t[4]=y-f*p,t[8]=_*p+m,t[1]=p,t[5]=o*u,t[9]=-a*u,t[2]=-c*u,t[6]=m*p+_,t[10]=f-y*p}else if(e.order==="XZY"){let f=o*l,m=o*c,_=a*l,y=a*c;t[0]=l*u,t[4]=-p,t[8]=c*u,t[1]=f*p+y,t[5]=o*u,t[9]=m*p-_,t[2]=_*p-m,t[6]=a*u,t[10]=y*p+f}return t[3]=0,t[7]=0,t[11]=0,t[12]=0,t[13]=0,t[14]=0,t[15]=1,this}makeRotationFromQuaternion(e){return this.compose(IS,e,LS)}lookAt(e,t,i){let r=this.elements;return ln.subVectors(e,t),ln.lengthSq()===0&&(ln.z=1),ln.normalize(),Vi.crossVectors(i,ln),Vi.lengthSq()===0&&(Math.abs(i.z)===1?ln.x+=1e-4:ln.z+=1e-4,ln.normalize(),Vi.crossVectors(i,ln)),Vi.normalize(),kl.crossVectors(ln,Vi),r[0]=Vi.x,r[4]=kl.x,r[8]=ln.x,r[1]=Vi.y,r[5]=kl.y,r[9]=ln.y,r[2]=Vi.z,r[6]=kl.z,r[10]=ln.z,this}multiply(e){return this.multiplyMatrices(this,e)}premultiply(e){return this.multiplyMatrices(e,this)}multiplyMatrices(e,t){let i=e.elements,r=t.elements,s=this.elements,o=i[0],a=i[4],l=i[8],c=i[12],u=i[1],p=i[5],f=i[9],m=i[13],_=i[2],y=i[6],d=i[10],h=i[14],g=i[3],v=i[7],x=i[11],R=i[15],C=r[0],T=r[4],L=r[8],E=r[12],M=r[1],P=r[5],W=r[9],k=r[13],G=r[2],$=r[6],V=r[10],K=r[14],z=r[3],he=r[7],ge=r[11],_e=r[15];return s[0]=o*C+a*M+l*G+c*z,s[4]=o*T+a*P+l*$+c*he,s[8]=o*L+a*W+l*V+c*ge,s[12]=o*E+a*k+l*K+c*_e,s[1]=u*C+p*M+f*G+m*z,s[5]=u*T+p*P+f*$+m*he,s[9]=u*L+p*W+f*V+m*ge,s[13]=u*E+p*k+f*K+m*_e,s[2]=_*C+y*M+d*G+h*z,s[6]=_*T+y*P+d*$+h*he,s[10]=_*L+y*W+d*V+h*ge,s[14]=_*E+y*k+d*K+h*_e,s[3]=g*C+v*M+x*G+R*z,s[7]=g*T+v*P+x*$+R*he,s[11]=g*L+v*W+x*V+R*ge,s[15]=g*E+v*k+x*K+R*_e,this}multiplyScalar(e){let t=this.elements;return t[0]*=e,t[4]*=e,t[8]*=e,t[12]*=e,t[1]*=e,t[5]*=e,t[9]*=e,t[13]*=e,t[2]*=e,t[6]*=e,t[10]*=e,t[14]*=e,t[3]*=e,t[7]*=e,t[11]*=e,t[15]*=e,this}determinant(){let e=this.elements,t=e[0],i=e[4],r=e[8],s=e[12],o=e[1],a=e[5],l=e[9],c=e[13],u=e[2],p=e[6],f=e[10],m=e[14],_=e[3],y=e[7],d=e[11],h=e[15];return _*(+s*l*p-r*c*p-s*a*f+i*c*f+r*a*m-i*l*m)+y*(+t*l*m-t*c*f+s*o*f-r*o*m+r*c*u-s*l*u)+d*(+t*c*p-t*a*m-s*o*p+i*o*m+s*a*u-i*c*u)+h*(-r*a*u-t*l*p+t*a*f+r*o*p-i*o*f+i*l*u)}transpose(){let e=this.elements,t;return t=e[1],e[1]=e[4],e[4]=t,t=e[2],e[2]=e[8],e[8]=t,t=e[6],e[6]=e[9],e[9]=t,t=e[3],e[3]=e[12],e[12]=t,t=e[7],e[7]=e[13],e[13]=t,t=e[11],e[11]=e[14],e[14]=t,this}setPosition(e,t,i){let r=this.elements;return e.isVector3?(r[12]=e.x,r[13]=e.y,r[14]=e.z):(r[12]=e,r[13]=t,r[14]=i),this}invert(){let e=this.elements,t=e[0],i=e[1],r=e[2],s=e[3],o=e[4],a=e[5],l=e[6],c=e[7],u=e[8],p=e[9],f=e[10],m=e[11],_=e[12],y=e[13],d=e[14],h=e[15],g=p*d*c-y*f*c+y*l*m-a*d*m-p*l*h+a*f*h,v=_*f*c-u*d*c-_*l*m+o*d*m+u*l*h-o*f*h,x=u*y*c-_*p*c+_*a*m-o*y*m-u*a*h+o*p*h,R=_*p*l-u*y*l-_*a*f+o*y*f+u*a*d-o*p*d,C=t*g+i*v+r*x+s*R;if(C===0)return this.set(0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0);let T=1/C;return e[0]=g*T,e[1]=(y*f*s-p*d*s-y*r*m+i*d*m+p*r*h-i*f*h)*T,e[2]=(a*d*s-y*l*s+y*r*c-i*d*c-a*r*h+i*l*h)*T,e[3]=(p*l*s-a*f*s-p*r*c+i*f*c+a*r*m-i*l*m)*T,e[4]=v*T,e[5]=(u*d*s-_*f*s+_*r*m-t*d*m-u*r*h+t*f*h)*T,e[6]=(_*l*s-o*d*s-_*r*c+t*d*c+o*r*h-t*l*h)*T,e[7]=(o*f*s-u*l*s+u*r*c-t*f*c-o*r*m+t*l*m)*T,e[8]=x*T,e[9]=(_*p*s-u*y*s-_*i*m+t*y*m+u*i*h-t*p*h)*T,e[10]=(o*y*s-_*a*s+_*i*c-t*y*c-o*i*h+t*a*h)*T,e[11]=(u*a*s-o*p*s-u*i*c+t*p*c+o*i*m-t*a*m)*T,e[12]=R*T,e[13]=(u*y*r-_*p*r+_*i*f-t*y*f-u*i*d+t*p*d)*T,e[14]=(_*a*r-o*y*r-_*i*l+t*y*l+o*i*d-t*a*d)*T,e[15]=(o*p*r-u*a*r+u*i*l-t*p*l-o*i*f+t*a*f)*T,this}scale(e){let t=this.elements,i=e.x,r=e.y,s=e.z;return t[0]*=i,t[4]*=r,t[8]*=s,t[1]*=i,t[5]*=r,t[9]*=s,t[2]*=i,t[6]*=r,t[10]*=s,t[3]*=i,t[7]*=r,t[11]*=s,this}getMaxScaleOnAxis(){let e=this.elements,t=e[0]*e[0]+e[1]*e[1]+e[2]*e[2],i=e[4]*e[4]+e[5]*e[5]+e[6]*e[6],r=e[8]*e[8]+e[9]*e[9]+e[10]*e[10];return Math.sqrt(Math.max(t,i,r))}makeTranslation(e,t,i){return e.isVector3?this.set(1,0,0,e.x,0,1,0,e.y,0,0,1,e.z,0,0,0,1):this.set(1,0,0,e,0,1,0,t,0,0,1,i,0,0,0,1),this}makeRotationX(e){let t=Math.cos(e),i=Math.sin(e);return this.set(1,0,0,0,0,t,-i,0,0,i,t,0,0,0,0,1),this}makeRotationY(e){let t=Math.cos(e),i=Math.sin(e);return this.set(t,0,i,0,0,1,0,0,-i,0,t,0,0,0,0,1),this}makeRotationZ(e){let t=Math.cos(e),i=Math.sin(e);return this.set(t,-i,0,0,i,t,0,0,0,0,1,0,0,0,0,1),this}makeRotationAxis(e,t){let i=Math.cos(t),r=Math.sin(t),s=1-i,o=e.x,a=e.y,l=e.z,c=s*o,u=s*a;return this.set(c*o+i,c*a-r*l,c*l+r*a,0,c*a+r*l,u*a+i,u*l-r*o,0,c*l-r*a,u*l+r*o,s*l*l+i,0,0,0,0,1),this}makeScale(e,t,i){return this.set(e,0,0,0,0,t,0,0,0,0,i,0,0,0,0,1),this}makeShear(e,t,i,r,s,o){return this.set(1,i,s,0,e,1,o,0,t,r,1,0,0,0,0,1),this}compose(e,t,i){let r=this.elements,s=t._x,o=t._y,a=t._z,l=t._w,c=s+s,u=o+o,p=a+a,f=s*c,m=s*u,_=s*p,y=o*u,d=o*p,h=a*p,g=l*c,v=l*u,x=l*p,R=i.x,C=i.y,T=i.z;return r[0]=(1-(y+h))*R,r[1]=(m+x)*R,r[2]=(_-v)*R,r[3]=0,r[4]=(m-x)*C,r[5]=(1-(f+h))*C,r[6]=(d+g)*C,r[7]=0,r[8]=(_+v)*T,r[9]=(d-g)*T,r[10]=(1-(f+y))*T,r[11]=0,r[12]=e.x,r[13]=e.y,r[14]=e.z,r[15]=1,this}decompose(e,t,i){let r=this.elements,s=ys.set(r[0],r[1],r[2]).length(),o=ys.set(r[4],r[5],r[6]).length(),a=ys.set(r[8],r[9],r[10]).length();this.determinant()<0&&(s=-s),e.x=r[12],e.y=r[13],e.z=r[14],Pn.copy(this);let c=1/s,u=1/o,p=1/a;return Pn.elements[0]*=c,Pn.elements[1]*=c,Pn.elements[2]*=c,Pn.elements[4]*=u,Pn.elements[5]*=u,Pn.elements[6]*=u,Pn.elements[8]*=p,Pn.elements[9]*=p,Pn.elements[10]*=p,t.setFromRotationMatrix(Pn),i.x=s,i.y=o,i.z=a,this}makePerspective(e,t,i,r,s,o,a=ui){let l=this.elements,c=2*s/(t-e),u=2*s/(i-r),p=(t+e)/(t-e),f=(i+r)/(i-r),m,_;if(a===ui)m=-(o+s)/(o-s),_=-2*o*s/(o-s);else if(a===hc)m=-o/(o-s),_=-o*s/(o-s);else throw new Error("THREE.Matrix4.makePerspective(): Invalid coordinate system: "+a);return l[0]=c,l[4]=0,l[8]=p,l[12]=0,l[1]=0,l[5]=u,l[9]=f,l[13]=0,l[2]=0,l[6]=0,l[10]=m,l[14]=_,l[3]=0,l[7]=0,l[11]=-1,l[15]=0,this}makeOrthographic(e,t,i,r,s,o,a=ui){let l=this.elements,c=1/(t-e),u=1/(i-r),p=1/(o-s),f=(t+e)*c,m=(i+r)*u,_,y;if(a===ui)_=(o+s)*p,y=-2*p;else if(a===hc)_=s*p,y=-1*p;else throw new Error("THREE.Matrix4.makeOrthographic(): Invalid coordinate system: "+a);return l[0]=2*c,l[4]=0,l[8]=0,l[12]=-f,l[1]=0,l[5]=2*u,l[9]=0,l[13]=-m,l[2]=0,l[6]=0,l[10]=y,l[14]=-_,l[3]=0,l[7]=0,l[11]=0,l[15]=1,this}equals(e){let t=this.elements,i=e.elements;for(let r=0;r<16;r++)if(t[r]!==i[r])return!1;return!0}fromArray(e,t=0){for(let i=0;i<16;i++)this.elements[i]=e[i+t];return this}toArray(e=[],t=0){let i=this.elements;return e[t]=i[0],e[t+1]=i[1],e[t+2]=i[2],e[t+3]=i[3],e[t+4]=i[4],e[t+5]=i[5],e[t+6]=i[6],e[t+7]=i[7],e[t+8]=i[8],e[t+9]=i[9],e[t+10]=i[10],e[t+11]=i[11],e[t+12]=i[12],e[t+13]=i[13],e[t+14]=i[14],e[t+15]=i[15],e}},ys=new D,Pn=new et,IS=new D(0,0,0),LS=new D(1,1,1),Vi=new D,kl=new D,ln=new D,R_=new et,P_=new Nn,Xn=class n{constructor(e=0,t=0,i=0,r=n.DEFAULT_ORDER){this.isEuler=!0,this._x=e,this._y=t,this._z=i,this._order=r}get x(){return this._x}set x(e){this._x=e,this._onChangeCallback()}get y(){return this._y}set y(e){this._y=e,this._onChangeCallback()}get z(){return this._z}set z(e){this._z=e,this._onChangeCallback()}get order(){return this._order}set order(e){this._order=e,this._onChangeCallback()}set(e,t,i,r=this._order){return this._x=e,this._y=t,this._z=i,this._order=r,this._onChangeCallback(),this}clone(){return new this.constructor(this._x,this._y,this._z,this._order)}copy(e){return this._x=e._x,this._y=e._y,this._z=e._z,this._order=e._order,this._onChangeCallback(),this}setFromRotationMatrix(e,t=this._order,i=!0){let r=e.elements,s=r[0],o=r[4],a=r[8],l=r[1],c=r[5],u=r[9],p=r[2],f=r[6],m=r[10];switch(t){case"XYZ":this._y=Math.asin(en(a,-1,1)),Math.abs(a)<.9999999?(this._x=Math.atan2(-u,m),this._z=Math.atan2(-o,s)):(this._x=Math.atan2(f,c),this._z=0);break;case"YXZ":this._x=Math.asin(-en(u,-1,1)),Math.abs(u)<.9999999?(this._y=Math.atan2(a,m),this._z=Math.atan2(l,c)):(this._y=Math.atan2(-p,s),this._z=0);break;case"ZXY":this._x=Math.asin(en(f,-1,1)),Math.abs(f)<.9999999?(this._y=Math.atan2(-p,m),this._z=Math.atan2(-o,c)):(this._y=0,this._z=Math.atan2(l,s));break;case"ZYX":this._y=Math.asin(-en(p,-1,1)),Math.abs(p)<.9999999?(this._x=Math.atan2(f,m),this._z=Math.atan2(l,s)):(this._x=0,this._z=Math.atan2(-o,c));break;case"YZX":this._z=Math.asin(en(l,-1,1)),Math.abs(l)<.9999999?(this._x=Math.atan2(-u,c),this._y=Math.atan2(-p,s)):(this._x=0,this._y=Math.atan2(a,m));break;case"XZY":this._z=Math.asin(-en(o,-1,1)),Math.abs(o)<.9999999?(this._x=Math.atan2(f,c),this._y=Math.atan2(a,s)):(this._x=Math.atan2(-u,m),this._y=0);break;default:console.warn("THREE.Euler: .setFromRotationMatrix() encountered an unknown order: "+t)}return this._order=t,i===!0&&this._onChangeCallback(),this}setFromQuaternion(e,t,i){return R_.makeRotationFromQuaternion(e),this.setFromRotationMatrix(R_,t,i)}setFromVector3(e,t=this._order){return this.set(e.x,e.y,e.z,t)}reorder(e){return P_.setFromEuler(this),this.setFromQuaternion(P_,e)}equals(e){return e._x===this._x&&e._y===this._y&&e._z===this._z&&e._order===this._order}fromArray(e){return this._x=e[0],this._y=e[1],this._z=e[2],e[3]!==void 0&&(this._order=e[3]),this._onChangeCallback(),this}toArray(e=[],t=0){return e[t]=this._x,e[t+1]=this._y,e[t+2]=this._z,e[t+3]=this._order,e}_onChange(e){return this._onChangeCallback=e,this}_onChangeCallback(){}*[Symbol.iterator](){yield this._x,yield this._y,yield this._z,yield this._order}};Xn.DEFAULT_ORDER="XYZ";var pc=class{constructor(){this.mask=1}set(e){this.mask=(1<<e|0)>>>0}enable(e){this.mask|=1<<e|0}enableAll(){this.mask=-1}toggle(e){this.mask^=1<<e|0}disable(e){this.mask&=~(1<<e|0)}disableAll(){this.mask=0}test(e){return(this.mask&e.mask)!==0}isEnabled(e){return(this.mask&(1<<e|0))!==0}},US=0,I_=new D,xs=new Nn,ii=new et,zl=new D,Ko=new D,NS=new D,DS=new Nn,L_=new D(1,0,0),U_=new D(0,1,0),N_=new D(0,0,1),D_={type:"added"},OS={type:"removed"},ws={type:"childadded",child:null},kd={type:"childremoved",child:null},Vt=class n extends Zi{constructor(){super(),this.isObject3D=!0,Object.defineProperty(this,"id",{value:US++}),this.uuid=la(),this.name="",this.type="Object3D",this.parent=null,this.children=[],this.up=n.DEFAULT_UP.clone();let e=new D,t=new Xn,i=new Nn,r=new D(1,1,1);function s(){i.setFromEuler(t,!1)}function o(){t.setFromQuaternion(i,void 0,!1)}t._onChange(s),i._onChange(o),Object.defineProperties(this,{position:{configurable:!0,enumerable:!0,value:e},rotation:{configurable:!0,enumerable:!0,value:t},quaternion:{configurable:!0,enumerable:!0,value:i},scale:{configurable:!0,enumerable:!0,value:r},modelViewMatrix:{value:new et},normalMatrix:{value:new Ue}}),this.matrix=new et,this.matrixWorld=new et,this.matrixAutoUpdate=n.DEFAULT_MATRIX_AUTO_UPDATE,this.matrixWorldAutoUpdate=n.DEFAULT_MATRIX_WORLD_AUTO_UPDATE,this.matrixWorldNeedsUpdate=!1,this.layers=new pc,this.visible=!0,this.castShadow=!1,this.receiveShadow=!1,this.frustumCulled=!0,this.renderOrder=0,this.animations=[],this.userData={}}onBeforeShadow(){}onAfterShadow(){}onBeforeRender(){}onAfterRender(){}applyMatrix4(e){this.matrixAutoUpdate&&this.updateMatrix(),this.matrix.premultiply(e),this.matrix.decompose(this.position,this.quaternion,this.scale)}applyQuaternion(e){return this.quaternion.premultiply(e),this}setRotationFromAxisAngle(e,t){this.quaternion.setFromAxisAngle(e,t)}setRotationFromEuler(e){this.quaternion.setFromEuler(e,!0)}setRotationFromMatrix(e){this.quaternion.setFromRotationMatrix(e)}setRotationFromQuaternion(e){this.quaternion.copy(e)}rotateOnAxis(e,t){return xs.setFromAxisAngle(e,t),this.quaternion.multiply(xs),this}rotateOnWorldAxis(e,t){return xs.setFromAxisAngle(e,t),this.quaternion.premultiply(xs),this}rotateX(e){return this.rotateOnAxis(L_,e)}rotateY(e){return this.rotateOnAxis(U_,e)}rotateZ(e){return this.rotateOnAxis(N_,e)}translateOnAxis(e,t){return I_.copy(e).applyQuaternion(this.quaternion),this.position.add(I_.multiplyScalar(t)),this}translateX(e){return this.translateOnAxis(L_,e)}translateY(e){return this.translateOnAxis(U_,e)}translateZ(e){return this.translateOnAxis(N_,e)}localToWorld(e){return this.updateWorldMatrix(!0,!1),e.applyMatrix4(this.matrixWorld)}worldToLocal(e){return this.updateWorldMatrix(!0,!1),e.applyMatrix4(ii.copy(this.matrixWorld).invert())}lookAt(e,t,i){e.isVector3?zl.copy(e):zl.set(e,t,i);let r=this.parent;this.updateWorldMatrix(!0,!1),Ko.setFromMatrixPosition(this.matrixWorld),this.isCamera||this.isLight?ii.lookAt(Ko,zl,this.up):ii.lookAt(zl,Ko,this.up),this.quaternion.setFromRotationMatrix(ii),r&&(ii.extractRotation(r.matrixWorld),xs.setFromRotationMatrix(ii),this.quaternion.premultiply(xs.invert()))}add(e){if(arguments.length>1){for(let t=0;t<arguments.length;t++)this.add(arguments[t]);return this}return e===this?(console.error("THREE.Object3D.add: object can't be added as a child of itself.",e),this):(e&&e.isObject3D?(e.removeFromParent(),e.parent=this,this.children.push(e),e.dispatchEvent(D_),ws.child=e,this.dispatchEvent(ws),ws.child=null):console.error("THREE.Object3D.add: object not an instance of THREE.Object3D.",e),this)}remove(e){if(arguments.length>1){for(let i=0;i<arguments.length;i++)this.remove(arguments[i]);return this}let t=this.children.indexOf(e);return t!==-1&&(e.parent=null,this.children.splice(t,1),e.dispatchEvent(OS),kd.child=e,this.dispatchEvent(kd),kd.child=null),this}removeFromParent(){let e=this.parent;return e!==null&&e.remove(this),this}clear(){return this.remove(...this.children)}attach(e){return this.updateWorldMatrix(!0,!1),ii.copy(this.matrixWorld).invert(),e.parent!==null&&(e.parent.updateWorldMatrix(!0,!1),ii.multiply(e.parent.matrixWorld)),e.applyMatrix4(ii),e.removeFromParent(),e.parent=this,this.children.push(e),e.updateWorldMatrix(!1,!0),e.dispatchEvent(D_),ws.child=e,this.dispatchEvent(ws),ws.child=null,this}getObjectById(e){return this.getObjectByProperty("id",e)}getObjectByName(e){return this.getObjectByProperty("name",e)}getObjectByProperty(e,t){if(this[e]===t)return this;for(let i=0,r=this.children.length;i<r;i++){let o=this.children[i].getObjectByProperty(e,t);if(o!==void 0)return o}}getObjectsByProperty(e,t,i=[]){this[e]===t&&i.push(this);let r=this.children;for(let s=0,o=r.length;s<o;s++)r[s].getObjectsByProperty(e,t,i);return i}getWorldPosition(e){return this.updateWorldMatrix(!0,!1),e.setFromMatrixPosition(this.matrixWorld)}getWorldQuaternion(e){return this.updateWorldMatrix(!0,!1),this.matrixWorld.decompose(Ko,e,NS),e}getWorldScale(e){return this.updateWorldMatrix(!0,!1),this.matrixWorld.decompose(Ko,DS,e),e}getWorldDirection(e){this.updateWorldMatrix(!0,!1);let t=this.matrixWorld.elements;return e.set(t[8],t[9],t[10]).normalize()}raycast(){}traverse(e){e(this);let t=this.children;for(let i=0,r=t.length;i<r;i++)t[i].traverse(e)}traverseVisible(e){if(this.visible===!1)return;e(this);let t=this.children;for(let i=0,r=t.length;i<r;i++)t[i].traverseVisible(e)}traverseAncestors(e){let t=this.parent;t!==null&&(e(t),t.traverseAncestors(e))}updateMatrix(){this.matrix.compose(this.position,this.quaternion,this.scale),this.matrixWorldNeedsUpdate=!0}updateMatrixWorld(e){this.matrixAutoUpdate&&this.updateMatrix(),(this.matrixWorldNeedsUpdate||e)&&(this.matrixWorldAutoUpdate===!0&&(this.parent===null?this.matrixWorld.copy(this.matrix):this.matrixWorld.multiplyMatrices(this.parent.matrixWorld,this.matrix)),this.matrixWorldNeedsUpdate=!1,e=!0);let t=this.children;for(let i=0,r=t.length;i<r;i++)t[i].updateMatrixWorld(e)}updateWorldMatrix(e,t){let i=this.parent;if(e===!0&&i!==null&&i.updateWorldMatrix(!0,!1),this.matrixAutoUpdate&&this.updateMatrix(),this.matrixWorldAutoUpdate===!0&&(this.parent===null?this.matrixWorld.copy(this.matrix):this.matrixWorld.multiplyMatrices(this.parent.matrixWorld,this.matrix)),t===!0){let r=this.children;for(let s=0,o=r.length;s<o;s++)r[s].updateWorldMatrix(!1,!0)}}toJSON(e){let t=e===void 0||typeof e=="string",i={};t&&(e={geometries:{},materials:{},textures:{},images:{},shapes:{},skeletons:{},animations:{},nodes:{}},i.metadata={version:4.6,type:"Object",generator:"Object3D.toJSON"});let r={};r.uuid=this.uuid,r.type=this.type,this.name!==""&&(r.name=this.name),this.castShadow===!0&&(r.castShadow=!0),this.receiveShadow===!0&&(r.receiveShadow=!0),this.visible===!1&&(r.visible=!1),this.frustumCulled===!1&&(r.frustumCulled=!1),this.renderOrder!==0&&(r.renderOrder=this.renderOrder),Object.keys(this.userData).length>0&&(r.userData=this.userData),r.layers=this.layers.mask,r.matrix=this.matrix.toArray(),r.up=this.up.toArray(),this.matrixAutoUpdate===!1&&(r.matrixAutoUpdate=!1),this.isInstancedMesh&&(r.type="InstancedMesh",r.count=this.count,r.instanceMatrix=this.instanceMatrix.toJSON(),this.instanceColor!==null&&(r.instanceColor=this.instanceColor.toJSON())),this.isBatchedMesh&&(r.type="BatchedMesh",r.perObjectFrustumCulled=this.perObjectFrustumCulled,r.sortObjects=this.sortObjects,r.drawRanges=this._drawRanges,r.reservedRanges=this._reservedRanges,r.visibility=this._visibility,r.active=this._active,r.bounds=this._bounds.map(a=>({boxInitialized:a.boxInitialized,boxMin:a.box.min.toArray(),boxMax:a.box.max.toArray(),sphereInitialized:a.sphereInitialized,sphereRadius:a.sphere.radius,sphereCenter:a.sphere.center.toArray()})),r.maxInstanceCount=this._maxInstanceCount,r.maxVertexCount=this._maxVertexCount,r.maxIndexCount=this._maxIndexCount,r.geometryInitialized=this._geometryInitialized,r.geometryCount=this._geometryCount,r.matricesTexture=this._matricesTexture.toJSON(e),this._colorsTexture!==null&&(r.colorsTexture=this._colorsTexture.toJSON(e)),this.boundingSphere!==null&&(r.boundingSphere={center:r.boundingSphere.center.toArray(),radius:r.boundingSphere.radius}),this.boundingBox!==null&&(r.boundingBox={min:r.boundingBox.min.toArray(),max:r.boundingBox.max.toArray()}));function s(a,l){return a[l.uuid]===void 0&&(a[l.uuid]=l.toJSON(e)),l.uuid}if(this.isScene)this.background&&(this.background.isColor?r.background=this.background.toJSON():this.background.isTexture&&(r.background=this.background.toJSON(e).uuid)),this.environment&&this.environment.isTexture&&this.environment.isRenderTargetTexture!==!0&&(r.environment=this.environment.toJSON(e).uuid);else if(this.isMesh||this.isLine||this.isPoints){r.geometry=s(e.geometries,this.geometry);let a=this.geometry.parameters;if(a!==void 0&&a.shapes!==void 0){let l=a.shapes;if(Array.isArray(l))for(let c=0,u=l.length;c<u;c++){let p=l[c];s(e.shapes,p)}else s(e.shapes,l)}}if(this.isSkinnedMesh&&(r.bindMode=this.bindMode,r.bindMatrix=this.bindMatrix.toArray(),this.skeleton!==void 0&&(s(e.skeletons,this.skeleton),r.skeleton=this.skeleton.uuid)),this.material!==void 0)if(Array.isArray(this.material)){let a=[];for(let l=0,c=this.material.length;l<c;l++)a.push(s(e.materials,this.material[l]));r.material=a}else r.material=s(e.materials,this.material);if(this.children.length>0){r.children=[];for(let a=0;a<this.children.length;a++)r.children.push(this.children[a].toJSON(e).object)}if(this.animations.length>0){r.animations=[];for(let a=0;a<this.animations.length;a++){let l=this.animations[a];r.animations.push(s(e.animations,l))}}if(t){let a=o(e.geometries),l=o(e.materials),c=o(e.textures),u=o(e.images),p=o(e.shapes),f=o(e.skeletons),m=o(e.animations),_=o(e.nodes);a.length>0&&(i.geometries=a),l.length>0&&(i.materials=l),c.length>0&&(i.textures=c),u.length>0&&(i.images=u),p.length>0&&(i.shapes=p),f.length>0&&(i.skeletons=f),m.length>0&&(i.animations=m),_.length>0&&(i.nodes=_)}return i.object=r,i;function o(a){let l=[];for(let c in a){let u=a[c];delete u.metadata,l.push(u)}return l}}clone(e){return new this.constructor().copy(this,e)}copy(e,t=!0){if(this.name=e.name,this.up.copy(e.up),this.position.copy(e.position),this.rotation.order=e.rotation.order,this.quaternion.copy(e.quaternion),this.scale.copy(e.scale),this.matrix.copy(e.matrix),this.matrixWorld.copy(e.matrixWorld),this.matrixAutoUpdate=e.matrixAutoUpdate,this.matrixWorldAutoUpdate=e.matrixWorldAutoUpdate,this.matrixWorldNeedsUpdate=e.matrixWorldNeedsUpdate,this.layers.mask=e.layers.mask,this.visible=e.visible,this.castShadow=e.castShadow,this.receiveShadow=e.receiveShadow,this.frustumCulled=e.frustumCulled,this.renderOrder=e.renderOrder,this.animations=e.animations.slice(),this.userData=JSON.parse(JSON.stringify(e.userData)),t===!0)for(let i=0;i<e.children.length;i++){let r=e.children[i];this.add(r.clone())}return this}};Vt.DEFAULT_UP=new D(0,1,0);Vt.DEFAULT_MATRIX_AUTO_UPDATE=!0;Vt.DEFAULT_MATRIX_WORLD_AUTO_UPDATE=!0;var In=new D,ri=new D,zd=new D,si=new D,Ss=new D,Ms=new D,O_=new D,Vd=new D,Hd=new D,Gd=new D,Is=class n{constructor(e=new D,t=new D,i=new D){this.a=e,this.b=t,this.c=i}static getNormal(e,t,i,r){r.subVectors(i,t),In.subVectors(e,t),r.cross(In);let s=r.lengthSq();return s>0?r.multiplyScalar(1/Math.sqrt(s)):r.set(0,0,0)}static getBarycoord(e,t,i,r,s){In.subVectors(r,t),ri.subVectors(i,t),zd.subVectors(e,t);let o=In.dot(In),a=In.dot(ri),l=In.dot(zd),c=ri.dot(ri),u=ri.dot(zd),p=o*c-a*a;if(p===0)return s.set(0,0,0),null;let f=1/p,m=(c*l-a*u)*f,_=(o*u-a*l)*f;return s.set(1-m-_,_,m)}static containsPoint(e,t,i,r){return this.getBarycoord(e,t,i,r,si)===null?!1:si.x>=0&&si.y>=0&&si.x+si.y<=1}static getInterpolation(e,t,i,r,s,o,a,l){return this.getBarycoord(e,t,i,r,si)===null?(l.x=0,l.y=0,"z"in l&&(l.z=0),"w"in l&&(l.w=0),null):(l.setScalar(0),l.addScaledVector(s,si.x),l.addScaledVector(o,si.y),l.addScaledVector(a,si.z),l)}static isFrontFacing(e,t,i,r){return In.subVectors(i,t),ri.subVectors(e,t),In.cross(ri).dot(r)<0}set(e,t,i){return this.a.copy(e),this.b.copy(t),this.c.copy(i),this}setFromPointsAndIndices(e,t,i,r){return this.a.copy(e[t]),this.b.copy(e[i]),this.c.copy(e[r]),this}setFromAttributeAndIndices(e,t,i,r){return this.a.fromBufferAttribute(e,t),this.b.fromBufferAttribute(e,i),this.c.fromBufferAttribute(e,r),this}clone(){return new this.constructor().copy(this)}copy(e){return this.a.copy(e.a),this.b.copy(e.b),this.c.copy(e.c),this}getArea(){return In.subVectors(this.c,this.b),ri.subVectors(this.a,this.b),In.cross(ri).length()*.5}getMidpoint(e){return e.addVectors(this.a,this.b).add(this.c).multiplyScalar(1/3)}getNormal(e){return n.getNormal(this.a,this.b,this.c,e)}getPlane(e){return e.setFromCoplanarPoints(this.a,this.b,this.c)}getBarycoord(e,t){return n.getBarycoord(e,this.a,this.b,this.c,t)}getInterpolation(e,t,i,r,s){return n.getInterpolation(e,this.a,this.b,this.c,t,i,r,s)}containsPoint(e){return n.containsPoint(e,this.a,this.b,this.c)}isFrontFacing(e){return n.isFrontFacing(this.a,this.b,this.c,e)}intersectsBox(e){return e.intersectsTriangle(this)}closestPointToPoint(e,t){let i=this.a,r=this.b,s=this.c,o,a;Ss.subVectors(r,i),Ms.subVectors(s,i),Vd.subVectors(e,i);let l=Ss.dot(Vd),c=Ms.dot(Vd);if(l<=0&&c<=0)return t.copy(i);Hd.subVectors(e,r);let u=Ss.dot(Hd),p=Ms.dot(Hd);if(u>=0&&p<=u)return t.copy(r);let f=l*p-u*c;if(f<=0&&l>=0&&u<=0)return o=l/(l-u),t.copy(i).addScaledVector(Ss,o);Gd.subVectors(e,s);let m=Ss.dot(Gd),_=Ms.dot(Gd);if(_>=0&&m<=_)return t.copy(s);let y=m*c-l*_;if(y<=0&&c>=0&&_<=0)return a=c/(c-_),t.copy(i).addScaledVector(Ms,a);let d=u*_-m*p;if(d<=0&&p-u>=0&&m-_>=0)return O_.subVectors(s,r),a=(p-u)/(p-u+(m-_)),t.copy(r).addScaledVector(O_,a);let h=1/(d+y+f);return o=y*h,a=f*h,t.copy(i).addScaledVector(Ss,o).addScaledVector(Ms,a)}equals(e){return e.a.equals(this.a)&&e.b.equals(this.b)&&e.c.equals(this.c)}},Lv={aliceblue:15792383,antiquewhite:16444375,aqua:65535,aquamarine:8388564,azure:15794175,beige:16119260,bisque:16770244,black:0,blanchedalmond:16772045,blue:255,blueviolet:9055202,brown:10824234,burlywood:14596231,cadetblue:6266528,chartreuse:8388352,chocolate:13789470,coral:16744272,cornflowerblue:6591981,cornsilk:16775388,crimson:14423100,cyan:65535,darkblue:139,darkcyan:35723,darkgoldenrod:12092939,darkgray:11119017,darkgreen:25600,darkgrey:11119017,darkkhaki:12433259,darkmagenta:9109643,darkolivegreen:5597999,darkorange:16747520,darkorchid:10040012,darkred:9109504,darksalmon:15308410,darkseagreen:9419919,darkslateblue:4734347,darkslategray:3100495,darkslategrey:3100495,darkturquoise:52945,darkviolet:9699539,deeppink:16716947,deepskyblue:49151,dimgray:6908265,dimgrey:6908265,dodgerblue:2003199,firebrick:11674146,floralwhite:16775920,forestgreen:2263842,fuchsia:16711935,gainsboro:14474460,ghostwhite:16316671,gold:16766720,goldenrod:14329120,gray:8421504,green:32768,greenyellow:11403055,grey:8421504,honeydew:15794160,hotpink:16738740,indianred:13458524,indigo:4915330,ivory:16777200,khaki:15787660,lavender:15132410,lavenderblush:16773365,lawngreen:8190976,lemonchiffon:16775885,lightblue:11393254,lightcoral:15761536,lightcyan:14745599,lightgoldenrodyellow:16448210,lightgray:13882323,lightgreen:9498256,lightgrey:13882323,lightpink:16758465,lightsalmon:16752762,lightseagreen:2142890,lightskyblue:8900346,lightslategray:7833753,lightslategrey:7833753,lightsteelblue:11584734,lightyellow:16777184,lime:65280,limegreen:3329330,linen:16445670,magenta:16711935,maroon:8388608,mediumaquamarine:6737322,mediumblue:205,mediumorchid:12211667,mediumpurple:9662683,mediumseagreen:3978097,mediumslateblue:8087790,mediumspringgreen:64154,mediumturquoise:4772300,mediumvioletred:13047173,midnightblue:1644912,mintcream:16121850,mistyrose:16770273,moccasin:16770229,navajowhite:16768685,navy:128,oldlace:16643558,olive:8421376,olivedrab:7048739,orange:16753920,orangered:16729344,orchid:14315734,palegoldenrod:15657130,palegreen:10025880,paleturquoise:11529966,palevioletred:14381203,papayawhip:16773077,peachpuff:16767673,peru:13468991,pink:16761035,plum:14524637,powderblue:11591910,purple:8388736,rebeccapurple:6697881,red:16711680,rosybrown:12357519,royalblue:4286945,saddlebrown:9127187,salmon:16416882,sandybrown:16032864,seagreen:3050327,seashell:16774638,sienna:10506797,silver:12632256,skyblue:8900331,slateblue:6970061,slategray:7372944,slategrey:7372944,snow:16775930,springgreen:65407,steelblue:4620980,tan:13808780,teal:32896,thistle:14204888,tomato:16737095,turquoise:4251856,violet:15631086,wheat:16113331,white:16777215,whitesmoke:16119285,yellow:16776960,yellowgreen:10145074},Hi={h:0,s:0,l:0},Vl={h:0,s:0,l:0};function Wd(n,e,t){return t<0&&(t+=1),t>1&&(t-=1),t<1/6?n+(e-n)*6*t:t<1/2?e:t<2/3?n+(e-n)*6*(2/3-t):n}var Oe=class{constructor(e,t,i){return this.isColor=!0,this.r=1,this.g=1,this.b=1,this.set(e,t,i)}set(e,t,i){if(t===void 0&&i===void 0){let r=e;r&&r.isColor?this.copy(r):typeof r=="number"?this.setHex(r):typeof r=="string"&&this.setStyle(r)}else this.setRGB(e,t,i);return this}setScalar(e){return this.r=e,this.g=e,this.b=e,this}setHex(e,t=un){return e=Math.floor(e),this.r=(e>>16&255)/255,this.g=(e>>8&255)/255,this.b=(e&255)/255,Ke.toWorkingColorSpace(this,t),this}setRGB(e,t,i,r=Ke.workingColorSpace){return this.r=e,this.g=t,this.b=i,Ke.toWorkingColorSpace(this,r),this}setHSL(e,t,i,r=Ke.workingColorSpace){if(e=ES(e,1),t=en(t,0,1),i=en(i,0,1),t===0)this.r=this.g=this.b=i;else{let s=i<=.5?i*(1+t):i+t-i*t,o=2*i-s;this.r=Wd(o,s,e+1/3),this.g=Wd(o,s,e),this.b=Wd(o,s,e-1/3)}return Ke.toWorkingColorSpace(this,r),this}setStyle(e,t=un){function i(s){s!==void 0&&parseFloat(s)<1&&console.warn("THREE.Color: Alpha component of "+e+" will be ignored.")}let r;if(r=/^(\w+)\(([^\)]*)\)/.exec(e)){let s,o=r[1],a=r[2];switch(o){case"rgb":case"rgba":if(s=/^\s*(\d+)\s*,\s*(\d+)\s*,\s*(\d+)\s*(?:,\s*(\d*\.?\d+)\s*)?$/.exec(a))return i(s[4]),this.setRGB(Math.min(255,parseInt(s[1],10))/255,Math.min(255,parseInt(s[2],10))/255,Math.min(255,parseInt(s[3],10))/255,t);if(s=/^\s*(\d+)\%\s*,\s*(\d+)\%\s*,\s*(\d+)\%\s*(?:,\s*(\d*\.?\d+)\s*)?$/.exec(a))return i(s[4]),this.setRGB(Math.min(100,parseInt(s[1],10))/100,Math.min(100,parseInt(s[2],10))/100,Math.min(100,parseInt(s[3],10))/100,t);break;case"hsl":case"hsla":if(s=/^\s*(\d*\.?\d+)\s*,\s*(\d*\.?\d+)\%\s*,\s*(\d*\.?\d+)\%\s*(?:,\s*(\d*\.?\d+)\s*)?$/.exec(a))return i(s[4]),this.setHSL(parseFloat(s[1])/360,parseFloat(s[2])/100,parseFloat(s[3])/100,t);break;default:console.warn("THREE.Color: Unknown color model "+e)}}else if(r=/^\#([A-Fa-f\d]+)$/.exec(e)){let s=r[1],o=s.length;if(o===3)return this.setRGB(parseInt(s.charAt(0),16)/15,parseInt(s.charAt(1),16)/15,parseInt(s.charAt(2),16)/15,t);if(o===6)return this.setHex(parseInt(s,16),t);console.warn("THREE.Color: Invalid hex color "+e)}else if(e&&e.length>0)return this.setColorName(e,t);return this}setColorName(e,t=un){let i=Lv[e.toLowerCase()];return i!==void 0?this.setHex(i,t):console.warn("THREE.Color: Unknown color "+e),this}clone(){return new this.constructor(this.r,this.g,this.b)}copy(e){return this.r=e.r,this.g=e.g,this.b=e.b,this}copySRGBToLinear(e){return this.r=Ds(e.r),this.g=Ds(e.g),this.b=Ds(e.b),this}copyLinearToSRGB(e){return this.r=Id(e.r),this.g=Id(e.g),this.b=Id(e.b),this}convertSRGBToLinear(){return this.copySRGBToLinear(this),this}convertLinearToSRGB(){return this.copyLinearToSRGB(this),this}getHex(e=un){return Ke.fromWorkingColorSpace(zt.copy(this),e),Math.round(en(zt.r*255,0,255))*65536+Math.round(en(zt.g*255,0,255))*256+Math.round(en(zt.b*255,0,255))}getHexString(e=un){return("000000"+this.getHex(e).toString(16)).slice(-6)}getHSL(e,t=Ke.workingColorSpace){Ke.fromWorkingColorSpace(zt.copy(this),t);let i=zt.r,r=zt.g,s=zt.b,o=Math.max(i,r,s),a=Math.min(i,r,s),l,c,u=(a+o)/2;if(a===o)l=0,c=0;else{let p=o-a;switch(c=u<=.5?p/(o+a):p/(2-o-a),o){case i:l=(r-s)/p+(r<s?6:0);break;case r:l=(s-i)/p+2;break;case s:l=(i-r)/p+4;break}l/=6}return e.h=l,e.s=c,e.l=u,e}getRGB(e,t=Ke.workingColorSpace){return Ke.fromWorkingColorSpace(zt.copy(this),t),e.r=zt.r,e.g=zt.g,e.b=zt.b,e}getStyle(e=un){Ke.fromWorkingColorSpace(zt.copy(this),e);let t=zt.r,i=zt.g,r=zt.b;return e!==un?`color(${e} ${t.toFixed(3)} ${i.toFixed(3)} ${r.toFixed(3)})`:`rgb(${Math.round(t*255)},${Math.round(i*255)},${Math.round(r*255)})`}offsetHSL(e,t,i){return this.getHSL(Hi),this.setHSL(Hi.h+e,Hi.s+t,Hi.l+i)}add(e){return this.r+=e.r,this.g+=e.g,this.b+=e.b,this}addColors(e,t){return this.r=e.r+t.r,this.g=e.g+t.g,this.b=e.b+t.b,this}addScalar(e){return this.r+=e,this.g+=e,this.b+=e,this}sub(e){return this.r=Math.max(0,this.r-e.r),this.g=Math.max(0,this.g-e.g),this.b=Math.max(0,this.b-e.b),this}multiply(e){return this.r*=e.r,this.g*=e.g,this.b*=e.b,this}multiplyScalar(e){return this.r*=e,this.g*=e,this.b*=e,this}lerp(e,t){return this.r+=(e.r-this.r)*t,this.g+=(e.g-this.g)*t,this.b+=(e.b-this.b)*t,this}lerpColors(e,t,i){return this.r=e.r+(t.r-e.r)*i,this.g=e.g+(t.g-e.g)*i,this.b=e.b+(t.b-e.b)*i,this}lerpHSL(e,t){this.getHSL(Hi),e.getHSL(Vl);let i=Rd(Hi.h,Vl.h,t),r=Rd(Hi.s,Vl.s,t),s=Rd(Hi.l,Vl.l,t);return this.setHSL(i,r,s),this}setFromVector3(e){return this.r=e.x,this.g=e.y,this.b=e.z,this}applyMatrix3(e){let t=this.r,i=this.g,r=this.b,s=e.elements;return this.r=s[0]*t+s[3]*i+s[6]*r,this.g=s[1]*t+s[4]*i+s[7]*r,this.b=s[2]*t+s[5]*i+s[8]*r,this}equals(e){return e.r===this.r&&e.g===this.g&&e.b===this.b}fromArray(e,t=0){return this.r=e[t],this.g=e[t+1],this.b=e[t+2],this}toArray(e=[],t=0){return e[t]=this.r,e[t+1]=this.g,e[t+2]=this.b,e}fromBufferAttribute(e,t){return this.r=e.getX(t),this.g=e.getY(t),this.b=e.getZ(t),this}toJSON(){return this.getHex()}*[Symbol.iterator](){yield this.r,yield this.g,yield this.b}},zt=new Oe;Oe.NAMES=Lv;var FS=0,Ar=class extends Zi{constructor(){super(),this.isMaterial=!0,Object.defineProperty(this,"id",{value:FS++}),this.uuid=la(),this.name="",this.type="Material",this.blending=Us,this.side=$i,this.vertexColors=!1,this.opacity=1,this.transparent=!1,this.alphaHash=!1,this.blendSrc=tf,this.blendDst=nf,this.blendEquation=Sr,this.blendSrcAlpha=null,this.blendDstAlpha=null,this.blendEquationAlpha=null,this.blendColor=new Oe(0,0,0),this.blendAlpha=0,this.depthFunc=oc,this.depthTest=!0,this.depthWrite=!0,this.stencilWriteMask=255,this.stencilFunc=S_,this.stencilRef=0,this.stencilFuncMask=255,this.stencilFail=ps,this.stencilZFail=ps,this.stencilZPass=ps,this.stencilWrite=!1,this.clippingPlanes=null,this.clipIntersection=!1,this.clipShadows=!1,this.shadowSide=null,this.colorWrite=!0,this.precision=null,this.polygonOffset=!1,this.polygonOffsetFactor=0,this.polygonOffsetUnits=0,this.dithering=!1,this.alphaToCoverage=!1,this.premultipliedAlpha=!1,this.forceSinglePass=!1,this.visible=!0,this.toneMapped=!0,this.userData={},this.version=0,this._alphaTest=0}get alphaTest(){return this._alphaTest}set alphaTest(e){this._alphaTest>0!=e>0&&this.version++,this._alphaTest=e}onBeforeCompile(){}customProgramCacheKey(){return this.onBeforeCompile.toString()}setValues(e){if(e!==void 0)for(let t in e){let i=e[t];if(i===void 0){console.warn(`THREE.Material: parameter '${t}' has value of undefined.`);continue}let r=this[t];if(r===void 0){console.warn(`THREE.Material: '${t}' is not a property of THREE.${this.type}.`);continue}r&&r.isColor?r.set(i):r&&r.isVector3&&i&&i.isVector3?r.copy(i):this[t]=i}}toJSON(e){let t=e===void 0||typeof e=="string";t&&(e={textures:{},images:{}});let i={metadata:{version:4.6,type:"Material",generator:"Material.toJSON"}};i.uuid=this.uuid,i.type=this.type,this.name!==""&&(i.name=this.name),this.color&&this.color.isColor&&(i.color=this.color.getHex()),this.roughness!==void 0&&(i.roughness=this.roughness),this.metalness!==void 0&&(i.metalness=this.metalness),this.sheen!==void 0&&(i.sheen=this.sheen),this.sheenColor&&this.sheenColor.isColor&&(i.sheenColor=this.sheenColor.getHex()),this.sheenRoughness!==void 0&&(i.sheenRoughness=this.sheenRoughness),this.emissive&&this.emissive.isColor&&(i.emissive=this.emissive.getHex()),this.emissiveIntensity!==void 0&&this.emissiveIntensity!==1&&(i.emissiveIntensity=this.emissiveIntensity),this.specular&&this.specular.isColor&&(i.specular=this.specular.getHex()),this.specularIntensity!==void 0&&(i.specularIntensity=this.specularIntensity),this.specularColor&&this.specularColor.isColor&&(i.specularColor=this.specularColor.getHex()),this.shininess!==void 0&&(i.shininess=this.shininess),this.clearcoat!==void 0&&(i.clearcoat=this.clearcoat),this.clearcoatRoughness!==void 0&&(i.clearcoatRoughness=this.clearcoatRoughness),this.clearcoatMap&&this.clearcoatMap.isTexture&&(i.clearcoatMap=this.clearcoatMap.toJSON(e).uuid),this.clearcoatRoughnessMap&&this.clearcoatRoughnessMap.isTexture&&(i.clearcoatRoughnessMap=this.clearcoatRoughnessMap.toJSON(e).uuid),this.clearcoatNormalMap&&this.clearcoatNormalMap.isTexture&&(i.clearcoatNormalMap=this.clearcoatNormalMap.toJSON(e).uuid,i.clearcoatNormalScale=this.clearcoatNormalScale.toArray()),this.dispersion!==void 0&&(i.dispersion=this.dispersion),this.iridescence!==void 0&&(i.iridescence=this.iridescence),this.iridescenceIOR!==void 0&&(i.iridescenceIOR=this.iridescenceIOR),this.iridescenceThicknessRange!==void 0&&(i.iridescenceThicknessRange=this.iridescenceThicknessRange),this.iridescenceMap&&this.iridescenceMap.isTexture&&(i.iridescenceMap=this.iridescenceMap.toJSON(e).uuid),this.iridescenceThicknessMap&&this.iridescenceThicknessMap.isTexture&&(i.iridescenceThicknessMap=this.iridescenceThicknessMap.toJSON(e).uuid),this.anisotropy!==void 0&&(i.anisotropy=this.anisotropy),this.anisotropyRotation!==void 0&&(i.anisotropyRotation=this.anisotropyRotation),this.anisotropyMap&&this.anisotropyMap.isTexture&&(i.anisotropyMap=this.anisotropyMap.toJSON(e).uuid),this.map&&this.map.isTexture&&(i.map=this.map.toJSON(e).uuid),this.matcap&&this.matcap.isTexture&&(i.matcap=this.matcap.toJSON(e).uuid),this.alphaMap&&this.alphaMap.isTexture&&(i.alphaMap=this.alphaMap.toJSON(e).uuid),this.lightMap&&this.lightMap.isTexture&&(i.lightMap=this.lightMap.toJSON(e).uuid,i.lightMapIntensity=this.lightMapIntensity),this.aoMap&&this.aoMap.isTexture&&(i.aoMap=this.aoMap.toJSON(e).uuid,i.aoMapIntensity=this.aoMapIntensity),this.bumpMap&&this.bumpMap.isTexture&&(i.bumpMap=this.bumpMap.toJSON(e).uuid,i.bumpScale=this.bumpScale),this.normalMap&&this.normalMap.isTexture&&(i.normalMap=this.normalMap.toJSON(e).uuid,i.normalMapType=this.normalMapType,i.normalScale=this.normalScale.toArray()),this.displacementMap&&this.displacementMap.isTexture&&(i.displacementMap=this.displacementMap.toJSON(e).uuid,i.displacementScale=this.displacementScale,i.displacementBias=this.displacementBias),this.roughnessMap&&this.roughnessMap.isTexture&&(i.roughnessMap=this.roughnessMap.toJSON(e).uuid),this.metalnessMap&&this.metalnessMap.isTexture&&(i.metalnessMap=this.metalnessMap.toJSON(e).uuid),this.emissiveMap&&this.emissiveMap.isTexture&&(i.emissiveMap=this.emissiveMap.toJSON(e).uuid),this.specularMap&&this.specularMap.isTexture&&(i.specularMap=this.specularMap.toJSON(e).uuid),this.specularIntensityMap&&this.specularIntensityMap.isTexture&&(i.specularIntensityMap=this.specularIntensityMap.toJSON(e).uuid),this.specularColorMap&&this.specularColorMap.isTexture&&(i.specularColorMap=this.specularColorMap.toJSON(e).uuid),this.envMap&&this.envMap.isTexture&&(i.envMap=this.envMap.toJSON(e).uuid,this.combine!==void 0&&(i.combine=this.combine)),this.envMapRotation!==void 0&&(i.envMapRotation=this.envMapRotation.toArray()),this.envMapIntensity!==void 0&&(i.envMapIntensity=this.envMapIntensity),this.reflectivity!==void 0&&(i.reflectivity=this.reflectivity),this.refractionRatio!==void 0&&(i.refractionRatio=this.refractionRatio),this.gradientMap&&this.gradientMap.isTexture&&(i.gradientMap=this.gradientMap.toJSON(e).uuid),this.transmission!==void 0&&(i.transmission=this.transmission),this.transmissionMap&&this.transmissionMap.isTexture&&(i.transmissionMap=this.transmissionMap.toJSON(e).uuid),this.thickness!==void 0&&(i.thickness=this.thickness),this.thicknessMap&&this.thicknessMap.isTexture&&(i.thicknessMap=this.thicknessMap.toJSON(e).uuid),this.attenuationDistance!==void 0&&this.attenuationDistance!==1/0&&(i.attenuationDistance=this.attenuationDistance),this.attenuationColor!==void 0&&(i.attenuationColor=this.attenuationColor.getHex()),this.size!==void 0&&(i.size=this.size),this.shadowSide!==null&&(i.shadowSide=this.shadowSide),this.sizeAttenuation!==void 0&&(i.sizeAttenuation=this.sizeAttenuation),this.blending!==Us&&(i.blending=this.blending),this.side!==$i&&(i.side=this.side),this.vertexColors===!0&&(i.vertexColors=!0),this.opacity<1&&(i.opacity=this.opacity),this.transparent===!0&&(i.transparent=!0),this.blendSrc!==tf&&(i.blendSrc=this.blendSrc),this.blendDst!==nf&&(i.blendDst=this.blendDst),this.blendEquation!==Sr&&(i.blendEquation=this.blendEquation),this.blendSrcAlpha!==null&&(i.blendSrcAlpha=this.blendSrcAlpha),this.blendDstAlpha!==null&&(i.blendDstAlpha=this.blendDstAlpha),this.blendEquationAlpha!==null&&(i.blendEquationAlpha=this.blendEquationAlpha),this.blendColor&&this.blendColor.isColor&&(i.blendColor=this.blendColor.getHex()),this.blendAlpha!==0&&(i.blendAlpha=this.blendAlpha),this.depthFunc!==oc&&(i.depthFunc=this.depthFunc),this.depthTest===!1&&(i.depthTest=this.depthTest),this.depthWrite===!1&&(i.depthWrite=this.depthWrite),this.colorWrite===!1&&(i.colorWrite=this.colorWrite),this.stencilWriteMask!==255&&(i.stencilWriteMask=this.stencilWriteMask),this.stencilFunc!==S_&&(i.stencilFunc=this.stencilFunc),this.stencilRef!==0&&(i.stencilRef=this.stencilRef),this.stencilFuncMask!==255&&(i.stencilFuncMask=this.stencilFuncMask),this.stencilFail!==ps&&(i.stencilFail=this.stencilFail),this.stencilZFail!==ps&&(i.stencilZFail=this.stencilZFail),this.stencilZPass!==ps&&(i.stencilZPass=this.stencilZPass),this.stencilWrite===!0&&(i.stencilWrite=this.stencilWrite),this.rotation!==void 0&&this.rotation!==0&&(i.rotation=this.rotation),this.polygonOffset===!0&&(i.polygonOffset=!0),this.polygonOffsetFactor!==0&&(i.polygonOffsetFactor=this.polygonOffsetFactor),this.polygonOffsetUnits!==0&&(i.polygonOffsetUnits=this.polygonOffsetUnits),this.linewidth!==void 0&&this.linewidth!==1&&(i.linewidth=this.linewidth),this.dashSize!==void 0&&(i.dashSize=this.dashSize),this.gapSize!==void 0&&(i.gapSize=this.gapSize),this.scale!==void 0&&(i.scale=this.scale),this.dithering===!0&&(i.dithering=!0),this.alphaTest>0&&(i.alphaTest=this.alphaTest),this.alphaHash===!0&&(i.alphaHash=!0),this.alphaToCoverage===!0&&(i.alphaToCoverage=!0),this.premultipliedAlpha===!0&&(i.premultipliedAlpha=!0),this.forceSinglePass===!0&&(i.forceSinglePass=!0),this.wireframe===!0&&(i.wireframe=!0),this.wireframeLinewidth>1&&(i.wireframeLinewidth=this.wireframeLinewidth),this.wireframeLinecap!=="round"&&(i.wireframeLinecap=this.wireframeLinecap),this.wireframeLinejoin!=="round"&&(i.wireframeLinejoin=this.wireframeLinejoin),this.flatShading===!0&&(i.flatShading=!0),this.visible===!1&&(i.visible=!1),this.toneMapped===!1&&(i.toneMapped=!1),this.fog===!1&&(i.fog=!1),Object.keys(this.userData).length>0&&(i.userData=this.userData);function r(s){let o=[];for(let a in s){let l=s[a];delete l.metadata,o.push(l)}return o}if(t){let s=r(e.textures),o=r(e.images);s.length>0&&(i.textures=s),o.length>0&&(i.images=o)}return i}clone(){return new this.constructor().copy(this)}copy(e){this.name=e.name,this.blending=e.blending,this.side=e.side,this.vertexColors=e.vertexColors,this.opacity=e.opacity,this.transparent=e.transparent,this.blendSrc=e.blendSrc,this.blendDst=e.blendDst,this.blendEquation=e.blendEquation,this.blendSrcAlpha=e.blendSrcAlpha,this.blendDstAlpha=e.blendDstAlpha,this.blendEquationAlpha=e.blendEquationAlpha,this.blendColor.copy(e.blendColor),this.blendAlpha=e.blendAlpha,this.depthFunc=e.depthFunc,this.depthTest=e.depthTest,this.depthWrite=e.depthWrite,this.stencilWriteMask=e.stencilWriteMask,this.stencilFunc=e.stencilFunc,this.stencilRef=e.stencilRef,this.stencilFuncMask=e.stencilFuncMask,this.stencilFail=e.stencilFail,this.stencilZFail=e.stencilZFail,this.stencilZPass=e.stencilZPass,this.stencilWrite=e.stencilWrite;let t=e.clippingPlanes,i=null;if(t!==null){let r=t.length;i=new Array(r);for(let s=0;s!==r;++s)i[s]=t[s].clone()}return this.clippingPlanes=i,this.clipIntersection=e.clipIntersection,this.clipShadows=e.clipShadows,this.shadowSide=e.shadowSide,this.colorWrite=e.colorWrite,this.precision=e.precision,this.polygonOffset=e.polygonOffset,this.polygonOffsetFactor=e.polygonOffsetFactor,this.polygonOffsetUnits=e.polygonOffsetUnits,this.dithering=e.dithering,this.alphaTest=e.alphaTest,this.alphaHash=e.alphaHash,this.alphaToCoverage=e.alphaToCoverage,this.premultipliedAlpha=e.premultipliedAlpha,this.forceSinglePass=e.forceSinglePass,this.visible=e.visible,this.toneMapped=e.toneMapped,this.userData=JSON.parse(JSON.stringify(e.userData)),this}dispose(){this.dispatchEvent({type:"dispose"})}set needsUpdate(e){e===!0&&this.version++}onBuild(){console.warn("Material: onBuild() has been removed.")}onBeforeRender(){console.warn("Material: onBeforeRender() has been removed.")}},Rr=class extends Ar{constructor(e){super(),this.isMeshBasicMaterial=!0,this.type="MeshBasicMaterial",this.color=new Oe(16777215),this.map=null,this.lightMap=null,this.lightMapIntensity=1,this.aoMap=null,this.aoMapIntensity=1,this.specularMap=null,this.alphaMap=null,this.envMap=null,this.envMapRotation=new Xn,this.combine=gv,this.reflectivity=1,this.refractionRatio=.98,this.wireframe=!1,this.wireframeLinewidth=1,this.wireframeLinecap="round",this.wireframeLinejoin="round",this.fog=!0,this.setValues(e)}copy(e){return super.copy(e),this.color.copy(e.color),this.map=e.map,this.lightMap=e.lightMap,this.lightMapIntensity=e.lightMapIntensity,this.aoMap=e.aoMap,this.aoMapIntensity=e.aoMapIntensity,this.specularMap=e.specularMap,this.alphaMap=e.alphaMap,this.envMap=e.envMap,this.envMapRotation.copy(e.envMapRotation),this.combine=e.combine,this.reflectivity=e.reflectivity,this.refractionRatio=e.refractionRatio,this.wireframe=e.wireframe,this.wireframeLinewidth=e.wireframeLinewidth,this.wireframeLinecap=e.wireframeLinecap,this.wireframeLinejoin=e.wireframeLinejoin,this.fog=e.fog,this}};var vt=new D,Hl=new ze,nn=class{constructor(e,t,i=!1){if(Array.isArray(e))throw new TypeError("THREE.BufferAttribute: array should be a Typed Array.");this.isBufferAttribute=!0,this.name="",this.array=e,this.itemSize=t,this.count=e!==void 0?e.length/t:0,this.normalized=i,this.usage=M_,this._updateRange={offset:0,count:-1},this.updateRanges=[],this.gpuType=ci,this.version=0}onUploadCallback(){}set needsUpdate(e){e===!0&&this.version++}get updateRange(){return Iv("THREE.BufferAttribute: updateRange() is deprecated and will be removed in r169. Use addUpdateRange() instead."),this._updateRange}setUsage(e){return this.usage=e,this}addUpdateRange(e,t){this.updateRanges.push({start:e,count:t})}clearUpdateRanges(){this.updateRanges.length=0}copy(e){return this.name=e.name,this.array=new e.array.constructor(e.array),this.itemSize=e.itemSize,this.count=e.count,this.normalized=e.normalized,this.usage=e.usage,this.gpuType=e.gpuType,this}copyAt(e,t,i){e*=this.itemSize,i*=t.itemSize;for(let r=0,s=this.itemSize;r<s;r++)this.array[e+r]=t.array[i+r];return this}copyArray(e){return this.array.set(e),this}applyMatrix3(e){if(this.itemSize===2)for(let t=0,i=this.count;t<i;t++)Hl.fromBufferAttribute(this,t),Hl.applyMatrix3(e),this.setXY(t,Hl.x,Hl.y);else if(this.itemSize===3)for(let t=0,i=this.count;t<i;t++)vt.fromBufferAttribute(this,t),vt.applyMatrix3(e),this.setXYZ(t,vt.x,vt.y,vt.z);return this}applyMatrix4(e){for(let t=0,i=this.count;t<i;t++)vt.fromBufferAttribute(this,t),vt.applyMatrix4(e),this.setXYZ(t,vt.x,vt.y,vt.z);return this}applyNormalMatrix(e){for(let t=0,i=this.count;t<i;t++)vt.fromBufferAttribute(this,t),vt.applyNormalMatrix(e),this.setXYZ(t,vt.x,vt.y,vt.z);return this}transformDirection(e){for(let t=0,i=this.count;t<i;t++)vt.fromBufferAttribute(this,t),vt.transformDirection(e),this.setXYZ(t,vt.x,vt.y,vt.z);return this}set(e,t=0){return this.array.set(e,t),this}getComponent(e,t){let i=this.array[e*this.itemSize+t];return this.normalized&&(i=$o(i,this.array)),i}setComponent(e,t,i){return this.normalized&&(i=Qt(i,this.array)),this.array[e*this.itemSize+t]=i,this}getX(e){let t=this.array[e*this.itemSize];return this.normalized&&(t=$o(t,this.array)),t}setX(e,t){return this.normalized&&(t=Qt(t,this.array)),this.array[e*this.itemSize]=t,this}getY(e){let t=this.array[e*this.itemSize+1];return this.normalized&&(t=$o(t,this.array)),t}setY(e,t){return this.normalized&&(t=Qt(t,this.array)),this.array[e*this.itemSize+1]=t,this}getZ(e){let t=this.array[e*this.itemSize+2];return this.normalized&&(t=$o(t,this.array)),t}setZ(e,t){return this.normalized&&(t=Qt(t,this.array)),this.array[e*this.itemSize+2]=t,this}getW(e){let t=this.array[e*this.itemSize+3];return this.normalized&&(t=$o(t,this.array)),t}setW(e,t){return this.normalized&&(t=Qt(t,this.array)),this.array[e*this.itemSize+3]=t,this}setXY(e,t,i){return e*=this.itemSize,this.normalized&&(t=Qt(t,this.array),i=Qt(i,this.array)),this.array[e+0]=t,this.array[e+1]=i,this}setXYZ(e,t,i,r){return e*=this.itemSize,this.normalized&&(t=Qt(t,this.array),i=Qt(i,this.array),r=Qt(r,this.array)),this.array[e+0]=t,this.array[e+1]=i,this.array[e+2]=r,this}setXYZW(e,t,i,r,s){return e*=this.itemSize,this.normalized&&(t=Qt(t,this.array),i=Qt(i,this.array),r=Qt(r,this.array),s=Qt(s,this.array)),this.array[e+0]=t,this.array[e+1]=i,this.array[e+2]=r,this.array[e+3]=s,this}onUpload(e){return this.onUploadCallback=e,this}clone(){return new this.constructor(this.array,this.itemSize).copy(this)}toJSON(){let e={itemSize:this.itemSize,type:this.array.constructor.name,array:Array.from(this.array),normalized:this.normalized};return this.name!==""&&(e.name=this.name),this.usage!==M_&&(e.usage=this.usage),e}};var mc=class extends nn{constructor(e,t,i){super(new Uint16Array(e),t,i)}};var gc=class extends nn{constructor(e,t,i){super(new Uint32Array(e),t,i)}};var hi=class extends nn{constructor(e,t,i){super(new Float32Array(e),t,i)}},BS=0,wn=new et,Xd=new Vt,Es=new D,cn=new br,jo=new br,Tt=new D,pi=class n extends Zi{constructor(){super(),this.isBufferGeometry=!0,Object.defineProperty(this,"id",{value:BS++}),this.uuid=la(),this.name="",this.type="BufferGeometry",this.index=null,this.attributes={},this.morphAttributes={},this.morphTargetsRelative=!1,this.groups=[],this.boundingBox=null,this.boundingSphere=null,this.drawRange={start:0,count:1/0},this.userData={}}getIndex(){return this.index}setIndex(e){return Array.isArray(e)?this.index=new(Pv(e)?gc:mc)(e,1):this.index=e,this}getAttribute(e){return this.attributes[e]}setAttribute(e,t){return this.attributes[e]=t,this}deleteAttribute(e){return delete this.attributes[e],this}hasAttribute(e){return this.attributes[e]!==void 0}addGroup(e,t,i=0){this.groups.push({start:e,count:t,materialIndex:i})}clearGroups(){this.groups=[]}setDrawRange(e,t){this.drawRange.start=e,this.drawRange.count=t}applyMatrix4(e){let t=this.attributes.position;t!==void 0&&(t.applyMatrix4(e),t.needsUpdate=!0);let i=this.attributes.normal;if(i!==void 0){let s=new Ue().getNormalMatrix(e);i.applyNormalMatrix(s),i.needsUpdate=!0}let r=this.attributes.tangent;return r!==void 0&&(r.transformDirection(e),r.needsUpdate=!0),this.boundingBox!==null&&this.computeBoundingBox(),this.boundingSphere!==null&&this.computeBoundingSphere(),this}applyQuaternion(e){return wn.makeRotationFromQuaternion(e),this.applyMatrix4(wn),this}rotateX(e){return wn.makeRotationX(e),this.applyMatrix4(wn),this}rotateY(e){return wn.makeRotationY(e),this.applyMatrix4(wn),this}rotateZ(e){return wn.makeRotationZ(e),this.applyMatrix4(wn),this}translate(e,t,i){return wn.makeTranslation(e,t,i),this.applyMatrix4(wn),this}scale(e,t,i){return wn.makeScale(e,t,i),this.applyMatrix4(wn),this}lookAt(e){return Xd.lookAt(e),Xd.updateMatrix(),this.applyMatrix4(Xd.matrix),this}center(){return this.computeBoundingBox(),this.boundingBox.getCenter(Es).negate(),this.translate(Es.x,Es.y,Es.z),this}setFromPoints(e){let t=[];for(let i=0,r=e.length;i<r;i++){let s=e[i];t.push(s.x,s.y,s.z||0)}return this.setAttribute("position",new hi(t,3)),this}computeBoundingBox(){this.boundingBox===null&&(this.boundingBox=new br);let e=this.attributes.position,t=this.morphAttributes.position;if(e&&e.isGLBufferAttribute){console.error("THREE.BufferGeometry.computeBoundingBox(): GLBufferAttribute requires a manual bounding box.",this),this.boundingBox.set(new D(-1/0,-1/0,-1/0),new D(1/0,1/0,1/0));return}if(e!==void 0){if(this.boundingBox.setFromBufferAttribute(e),t)for(let i=0,r=t.length;i<r;i++){let s=t[i];cn.setFromBufferAttribute(s),this.morphTargetsRelative?(Tt.addVectors(this.boundingBox.min,cn.min),this.boundingBox.expandByPoint(Tt),Tt.addVectors(this.boundingBox.max,cn.max),this.boundingBox.expandByPoint(Tt)):(this.boundingBox.expandByPoint(cn.min),this.boundingBox.expandByPoint(cn.max))}}else this.boundingBox.makeEmpty();(isNaN(this.boundingBox.min.x)||isNaN(this.boundingBox.min.y)||isNaN(this.boundingBox.min.z))&&console.error('THREE.BufferGeometry.computeBoundingBox(): Computed min/max have NaN values. The "position" attribute is likely to have NaN values.',this)}computeBoundingSphere(){this.boundingSphere===null&&(this.boundingSphere=new ia);let e=this.attributes.position,t=this.morphAttributes.position;if(e&&e.isGLBufferAttribute){console.error("THREE.BufferGeometry.computeBoundingSphere(): GLBufferAttribute requires a manual bounding sphere.",this),this.boundingSphere.set(new D,1/0);return}if(e){let i=this.boundingSphere.center;if(cn.setFromBufferAttribute(e),t)for(let s=0,o=t.length;s<o;s++){let a=t[s];jo.setFromBufferAttribute(a),this.morphTargetsRelative?(Tt.addVectors(cn.min,jo.min),cn.expandByPoint(Tt),Tt.addVectors(cn.max,jo.max),cn.expandByPoint(Tt)):(cn.expandByPoint(jo.min),cn.expandByPoint(jo.max))}cn.getCenter(i);let r=0;for(let s=0,o=e.count;s<o;s++)Tt.fromBufferAttribute(e,s),r=Math.max(r,i.distanceToSquared(Tt));if(t)for(let s=0,o=t.length;s<o;s++){let a=t[s],l=this.morphTargetsRelative;for(let c=0,u=a.count;c<u;c++)Tt.fromBufferAttribute(a,c),l&&(Es.fromBufferAttribute(e,c),Tt.add(Es)),r=Math.max(r,i.distanceToSquared(Tt))}this.boundingSphere.radius=Math.sqrt(r),isNaN(this.boundingSphere.radius)&&console.error('THREE.BufferGeometry.computeBoundingSphere(): Computed radius is NaN. The "position" attribute is likely to have NaN values.',this)}}computeTangents(){let e=this.index,t=this.attributes;if(e===null||t.position===void 0||t.normal===void 0||t.uv===void 0){console.error("THREE.BufferGeometry: .computeTangents() failed. Missing required attributes (index, position, normal or uv)");return}let i=t.position,r=t.normal,s=t.uv;this.hasAttribute("tangent")===!1&&this.setAttribute("tangent",new nn(new Float32Array(4*i.count),4));let o=this.getAttribute("tangent"),a=[],l=[];for(let L=0;L<i.count;L++)a[L]=new D,l[L]=new D;let c=new D,u=new D,p=new D,f=new ze,m=new ze,_=new ze,y=new D,d=new D;function h(L,E,M){c.fromBufferAttribute(i,L),u.fromBufferAttribute(i,E),p.fromBufferAttribute(i,M),f.fromBufferAttribute(s,L),m.fromBufferAttribute(s,E),_.fromBufferAttribute(s,M),u.sub(c),p.sub(c),m.sub(f),_.sub(f);let P=1/(m.x*_.y-_.x*m.y);isFinite(P)&&(y.copy(u).multiplyScalar(_.y).addScaledVector(p,-m.y).multiplyScalar(P),d.copy(p).multiplyScalar(m.x).addScaledVector(u,-_.x).multiplyScalar(P),a[L].add(y),a[E].add(y),a[M].add(y),l[L].add(d),l[E].add(d),l[M].add(d))}let g=this.groups;g.length===0&&(g=[{start:0,count:e.count}]);for(let L=0,E=g.length;L<E;++L){let M=g[L],P=M.start,W=M.count;for(let k=P,G=P+W;k<G;k+=3)h(e.getX(k+0),e.getX(k+1),e.getX(k+2))}let v=new D,x=new D,R=new D,C=new D;function T(L){R.fromBufferAttribute(r,L),C.copy(R);let E=a[L];v.copy(E),v.sub(R.multiplyScalar(R.dot(E))).normalize(),x.crossVectors(C,E);let P=x.dot(l[L])<0?-1:1;o.setXYZW(L,v.x,v.y,v.z,P)}for(let L=0,E=g.length;L<E;++L){let M=g[L],P=M.start,W=M.count;for(let k=P,G=P+W;k<G;k+=3)T(e.getX(k+0)),T(e.getX(k+1)),T(e.getX(k+2))}}computeVertexNormals(){let e=this.index,t=this.getAttribute("position");if(t!==void 0){let i=this.getAttribute("normal");if(i===void 0)i=new nn(new Float32Array(t.count*3),3),this.setAttribute("normal",i);else for(let f=0,m=i.count;f<m;f++)i.setXYZ(f,0,0,0);let r=new D,s=new D,o=new D,a=new D,l=new D,c=new D,u=new D,p=new D;if(e)for(let f=0,m=e.count;f<m;f+=3){let _=e.getX(f+0),y=e.getX(f+1),d=e.getX(f+2);r.fromBufferAttribute(t,_),s.fromBufferAttribute(t,y),o.fromBufferAttribute(t,d),u.subVectors(o,s),p.subVectors(r,s),u.cross(p),a.fromBufferAttribute(i,_),l.fromBufferAttribute(i,y),c.fromBufferAttribute(i,d),a.add(u),l.add(u),c.add(u),i.setXYZ(_,a.x,a.y,a.z),i.setXYZ(y,l.x,l.y,l.z),i.setXYZ(d,c.x,c.y,c.z)}else for(let f=0,m=t.count;f<m;f+=3)r.fromBufferAttribute(t,f+0),s.fromBufferAttribute(t,f+1),o.fromBufferAttribute(t,f+2),u.subVectors(o,s),p.subVectors(r,s),u.cross(p),i.setXYZ(f+0,u.x,u.y,u.z),i.setXYZ(f+1,u.x,u.y,u.z),i.setXYZ(f+2,u.x,u.y,u.z);this.normalizeNormals(),i.needsUpdate=!0}}normalizeNormals(){let e=this.attributes.normal;for(let t=0,i=e.count;t<i;t++)Tt.fromBufferAttribute(e,t),Tt.normalize(),e.setXYZ(t,Tt.x,Tt.y,Tt.z)}toNonIndexed(){function e(a,l){let c=a.array,u=a.itemSize,p=a.normalized,f=new c.constructor(l.length*u),m=0,_=0;for(let y=0,d=l.length;y<d;y++){a.isInterleavedBufferAttribute?m=l[y]*a.data.stride+a.offset:m=l[y]*u;for(let h=0;h<u;h++)f[_++]=c[m++]}return new nn(f,u,p)}if(this.index===null)return console.warn("THREE.BufferGeometry.toNonIndexed(): BufferGeometry is already non-indexed."),this;let t=new n,i=this.index.array,r=this.attributes;for(let a in r){let l=r[a],c=e(l,i);t.setAttribute(a,c)}let s=this.morphAttributes;for(let a in s){let l=[],c=s[a];for(let u=0,p=c.length;u<p;u++){let f=c[u],m=e(f,i);l.push(m)}t.morphAttributes[a]=l}t.morphTargetsRelative=this.morphTargetsRelative;let o=this.groups;for(let a=0,l=o.length;a<l;a++){let c=o[a];t.addGroup(c.start,c.count,c.materialIndex)}return t}toJSON(){let e={metadata:{version:4.6,type:"BufferGeometry",generator:"BufferGeometry.toJSON"}};if(e.uuid=this.uuid,e.type=this.type,this.name!==""&&(e.name=this.name),Object.keys(this.userData).length>0&&(e.userData=this.userData),this.parameters!==void 0){let l=this.parameters;for(let c in l)l[c]!==void 0&&(e[c]=l[c]);return e}e.data={attributes:{}};let t=this.index;t!==null&&(e.data.index={type:t.array.constructor.name,array:Array.prototype.slice.call(t.array)});let i=this.attributes;for(let l in i){let c=i[l];e.data.attributes[l]=c.toJSON(e.data)}let r={},s=!1;for(let l in this.morphAttributes){let c=this.morphAttributes[l],u=[];for(let p=0,f=c.length;p<f;p++){let m=c[p];u.push(m.toJSON(e.data))}u.length>0&&(r[l]=u,s=!0)}s&&(e.data.morphAttributes=r,e.data.morphTargetsRelative=this.morphTargetsRelative);let o=this.groups;o.length>0&&(e.data.groups=JSON.parse(JSON.stringify(o)));let a=this.boundingSphere;return a!==null&&(e.data.boundingSphere={center:a.center.toArray(),radius:a.radius}),e}clone(){return new this.constructor().copy(this)}copy(e){this.index=null,this.attributes={},this.morphAttributes={},this.groups=[],this.boundingBox=null,this.boundingSphere=null;let t={};this.name=e.name;let i=e.index;i!==null&&this.setIndex(i.clone(t));let r=e.attributes;for(let c in r){let u=r[c];this.setAttribute(c,u.clone(t))}let s=e.morphAttributes;for(let c in s){let u=[],p=s[c];for(let f=0,m=p.length;f<m;f++)u.push(p[f].clone(t));this.morphAttributes[c]=u}this.morphTargetsRelative=e.morphTargetsRelative;let o=e.groups;for(let c=0,u=o.length;c<u;c++){let p=o[c];this.addGroup(p.start,p.count,p.materialIndex)}let a=e.boundingBox;a!==null&&(this.boundingBox=a.clone());let l=e.boundingSphere;return l!==null&&(this.boundingSphere=l.clone()),this.drawRange.start=e.drawRange.start,this.drawRange.count=e.drawRange.count,this.userData=e.userData,this}dispose(){this.dispatchEvent({type:"dispose"})}},F_=new et,_r=new kf,Gl=new ia,B_=new D,Cs=new D,Ts=new D,bs=new D,Yd=new D,Wl=new D,Xl=new ze,Yl=new ze,ql=new ze,k_=new D,z_=new D,V_=new D,$l=new D,Zl=new D,St=class extends Vt{constructor(e=new pi,t=new Rr){super(),this.isMesh=!0,this.type="Mesh",this.geometry=e,this.material=t,this.updateMorphTargets()}copy(e,t){return super.copy(e,t),e.morphTargetInfluences!==void 0&&(this.morphTargetInfluences=e.morphTargetInfluences.slice()),e.morphTargetDictionary!==void 0&&(this.morphTargetDictionary=Object.assign({},e.morphTargetDictionary)),this.material=Array.isArray(e.material)?e.material.slice():e.material,this.geometry=e.geometry,this}updateMorphTargets(){let t=this.geometry.morphAttributes,i=Object.keys(t);if(i.length>0){let r=t[i[0]];if(r!==void 0){this.morphTargetInfluences=[],this.morphTargetDictionary={};for(let s=0,o=r.length;s<o;s++){let a=r[s].name||String(s);this.morphTargetInfluences.push(0),this.morphTargetDictionary[a]=s}}}}getVertexPosition(e,t){let i=this.geometry,r=i.attributes.position,s=i.morphAttributes.position,o=i.morphTargetsRelative;t.fromBufferAttribute(r,e);let a=this.morphTargetInfluences;if(s&&a){Wl.set(0,0,0);for(let l=0,c=s.length;l<c;l++){let u=a[l],p=s[l];u!==0&&(Yd.fromBufferAttribute(p,e),o?Wl.addScaledVector(Yd,u):Wl.addScaledVector(Yd.sub(t),u))}t.add(Wl)}return t}raycast(e,t){let i=this.geometry,r=this.material,s=this.matrixWorld;r!==void 0&&(i.boundingSphere===null&&i.computeBoundingSphere(),Gl.copy(i.boundingSphere),Gl.applyMatrix4(s),_r.copy(e.ray).recast(e.near),!(Gl.containsPoint(_r.origin)===!1&&(_r.intersectSphere(Gl,B_)===null||_r.origin.distanceToSquared(B_)>(e.far-e.near)**2))&&(F_.copy(s).invert(),_r.copy(e.ray).applyMatrix4(F_),!(i.boundingBox!==null&&_r.intersectsBox(i.boundingBox)===!1)&&this._computeIntersections(e,t,_r)))}_computeIntersections(e,t,i){let r,s=this.geometry,o=this.material,a=s.index,l=s.attributes.position,c=s.attributes.uv,u=s.attributes.uv1,p=s.attributes.normal,f=s.groups,m=s.drawRange;if(a!==null)if(Array.isArray(o))for(let _=0,y=f.length;_<y;_++){let d=f[_],h=o[d.materialIndex],g=Math.max(d.start,m.start),v=Math.min(a.count,Math.min(d.start+d.count,m.start+m.count));for(let x=g,R=v;x<R;x+=3){let C=a.getX(x),T=a.getX(x+1),L=a.getX(x+2);r=Jl(this,h,e,i,c,u,p,C,T,L),r&&(r.faceIndex=Math.floor(x/3),r.face.materialIndex=d.materialIndex,t.push(r))}}else{let _=Math.max(0,m.start),y=Math.min(a.count,m.start+m.count);for(let d=_,h=y;d<h;d+=3){let g=a.getX(d),v=a.getX(d+1),x=a.getX(d+2);r=Jl(this,o,e,i,c,u,p,g,v,x),r&&(r.faceIndex=Math.floor(d/3),t.push(r))}}else if(l!==void 0)if(Array.isArray(o))for(let _=0,y=f.length;_<y;_++){let d=f[_],h=o[d.materialIndex],g=Math.max(d.start,m.start),v=Math.min(l.count,Math.min(d.start+d.count,m.start+m.count));for(let x=g,R=v;x<R;x+=3){let C=x,T=x+1,L=x+2;r=Jl(this,h,e,i,c,u,p,C,T,L),r&&(r.faceIndex=Math.floor(x/3),r.face.materialIndex=d.materialIndex,t.push(r))}}else{let _=Math.max(0,m.start),y=Math.min(l.count,m.start+m.count);for(let d=_,h=y;d<h;d+=3){let g=d,v=d+1,x=d+2;r=Jl(this,o,e,i,c,u,p,g,v,x),r&&(r.faceIndex=Math.floor(d/3),t.push(r))}}}};function kS(n,e,t,i,r,s,o,a){let l;if(e.side===tn?l=i.intersectTriangle(o,s,r,!0,a):l=i.intersectTriangle(r,s,o,e.side===$i,a),l===null)return null;Zl.copy(a),Zl.applyMatrix4(n.matrixWorld);let c=t.ray.origin.distanceTo(Zl);return c<t.near||c>t.far?null:{distance:c,point:Zl.clone(),object:n}}function Jl(n,e,t,i,r,s,o,a,l,c){n.getVertexPosition(a,Cs),n.getVertexPosition(l,Ts),n.getVertexPosition(c,bs);let u=kS(n,e,t,i,Cs,Ts,bs,$l);if(u){r&&(Xl.fromBufferAttribute(r,a),Yl.fromBufferAttribute(r,l),ql.fromBufferAttribute(r,c),u.uv=Is.getInterpolation($l,Cs,Ts,bs,Xl,Yl,ql,new ze)),s&&(Xl.fromBufferAttribute(s,a),Yl.fromBufferAttribute(s,l),ql.fromBufferAttribute(s,c),u.uv1=Is.getInterpolation($l,Cs,Ts,bs,Xl,Yl,ql,new ze)),o&&(k_.fromBufferAttribute(o,a),z_.fromBufferAttribute(o,l),V_.fromBufferAttribute(o,c),u.normal=Is.getInterpolation($l,Cs,Ts,bs,k_,z_,V_,new D),u.normal.dot(i.direction)>0&&u.normal.multiplyScalar(-1));let p={a,b:l,c,normal:new D,materialIndex:0};Is.getNormal(Cs,Ts,bs,p.normal),u.face=p}return u}var Pr=class n extends pi{constructor(e=1,t=1,i=1,r=1,s=1,o=1){super(),this.type="BoxGeometry",this.parameters={width:e,height:t,depth:i,widthSegments:r,heightSegments:s,depthSegments:o};let a=this;r=Math.floor(r),s=Math.floor(s),o=Math.floor(o);let l=[],c=[],u=[],p=[],f=0,m=0;_("z","y","x",-1,-1,i,t,e,o,s,0),_("z","y","x",1,-1,i,t,-e,o,s,1),_("x","z","y",1,1,e,i,t,r,o,2),_("x","z","y",1,-1,e,i,-t,r,o,3),_("x","y","z",1,-1,e,t,i,r,s,4),_("x","y","z",-1,-1,e,t,-i,r,s,5),this.setIndex(l),this.setAttribute("position",new hi(c,3)),this.setAttribute("normal",new hi(u,3)),this.setAttribute("uv",new hi(p,2));function _(y,d,h,g,v,x,R,C,T,L,E){let M=x/T,P=R/L,W=x/2,k=R/2,G=C/2,$=T+1,V=L+1,K=0,z=0,he=new D;for(let ge=0;ge<V;ge++){let _e=ge*P-k;for(let Ve=0;Ve<$;Ve++){let je=Ve*M-W;he[y]=je*g,he[d]=_e*v,he[h]=G,c.push(he.x,he.y,he.z),he[y]=0,he[d]=0,he[h]=C>0?1:-1,u.push(he.x,he.y,he.z),p.push(Ve/T),p.push(1-ge/L),K+=1}}for(let ge=0;ge<L;ge++)for(let _e=0;_e<T;_e++){let Ve=f+_e+$*ge,je=f+_e+$*(ge+1),H=f+(_e+1)+$*(ge+1),Q=f+(_e+1)+$*ge;l.push(Ve,je,Q),l.push(je,H,Q),z+=6}a.addGroup(m,z,E),m+=z,f+=K}}copy(e){return super.copy(e),this.parameters=Object.assign({},e.parameters),this}static fromJSON(e){return new n(e.width,e.height,e.depth,e.widthSegments,e.heightSegments,e.depthSegments)}};function Vs(n){let e={};for(let t in n){e[t]={};for(let i in n[t]){let r=n[t][i];r&&(r.isColor||r.isMatrix3||r.isMatrix4||r.isVector2||r.isVector3||r.isVector4||r.isTexture||r.isQuaternion)?r.isRenderTargetTexture?(console.warn("UniformsUtils: Textures of render targets cannot be cloned via cloneUniforms() or mergeUniforms()."),e[t][i]=null):e[t][i]=r.clone():Array.isArray(r)?e[t][i]=r.slice():e[t][i]=r}}return e}function Xt(n){let e={};for(let t=0;t<n.length;t++){let i=Vs(n[t]);for(let r in i)e[r]=i[r]}return e}function zS(n){let e=[];for(let t=0;t<n.length;t++)e.push(n[t].clone());return e}function Uv(n){let e=n.getRenderTarget();return e===null?n.outputColorSpace:e.isXRRenderTarget===!0?e.texture.colorSpace:Ke.workingColorSpace}var VS={clone:Vs,merge:Xt},HS=`void main() {
	gl_Position = projectionMatrix * modelViewMatrix * vec4( position, 1.0 );
}`,GS=`void main() {
	gl_FragColor = vec4( 1.0, 0.0, 0.0, 1.0 );
}`,Mn=class extends Ar{constructor(e){super(),this.isShaderMaterial=!0,this.type="ShaderMaterial",this.defines={},this.uniforms={},this.uniformsGroups=[],this.vertexShader=HS,this.fragmentShader=GS,this.linewidth=1,this.wireframe=!1,this.wireframeLinewidth=1,this.fog=!1,this.lights=!1,this.clipping=!1,this.forceSinglePass=!0,this.extensions={clipCullDistance:!1,multiDraw:!1},this.defaultAttributeValues={color:[1,1,1],uv:[0,0],uv1:[0,0]},this.index0AttributeName=void 0,this.uniformsNeedUpdate=!1,this.glslVersion=null,e!==void 0&&this.setValues(e)}copy(e){return super.copy(e),this.fragmentShader=e.fragmentShader,this.vertexShader=e.vertexShader,this.uniforms=Vs(e.uniforms),this.uniformsGroups=zS(e.uniformsGroups),this.defines=Object.assign({},e.defines),this.wireframe=e.wireframe,this.wireframeLinewidth=e.wireframeLinewidth,this.fog=e.fog,this.lights=e.lights,this.clipping=e.clipping,this.extensions=Object.assign({},e.extensions),this.glslVersion=e.glslVersion,this}toJSON(e){let t=super.toJSON(e);t.glslVersion=this.glslVersion,t.uniforms={};for(let r in this.uniforms){let o=this.uniforms[r].value;o&&o.isTexture?t.uniforms[r]={type:"t",value:o.toJSON(e).uuid}:o&&o.isColor?t.uniforms[r]={type:"c",value:o.getHex()}:o&&o.isVector2?t.uniforms[r]={type:"v2",value:o.toArray()}:o&&o.isVector3?t.uniforms[r]={type:"v3",value:o.toArray()}:o&&o.isVector4?t.uniforms[r]={type:"v4",value:o.toArray()}:o&&o.isMatrix3?t.uniforms[r]={type:"m3",value:o.toArray()}:o&&o.isMatrix4?t.uniforms[r]={type:"m4",value:o.toArray()}:t.uniforms[r]={value:o}}Object.keys(this.defines).length>0&&(t.defines=this.defines),t.vertexShader=this.vertexShader,t.fragmentShader=this.fragmentShader,t.lights=this.lights,t.clipping=this.clipping;let i={};for(let r in this.extensions)this.extensions[r]===!0&&(i[r]=!0);return Object.keys(i).length>0&&(t.extensions=i),t}},_c=class extends Vt{constructor(){super(),this.isCamera=!0,this.type="Camera",this.matrixWorldInverse=new et,this.projectionMatrix=new et,this.projectionMatrixInverse=new et,this.coordinateSystem=ui}copy(e,t){return super.copy(e,t),this.matrixWorldInverse.copy(e.matrixWorldInverse),this.projectionMatrix.copy(e.projectionMatrix),this.projectionMatrixInverse.copy(e.projectionMatrixInverse),this.coordinateSystem=e.coordinateSystem,this}getWorldDirection(e){return super.getWorldDirection(e).negate()}updateMatrixWorld(e){super.updateMatrixWorld(e),this.matrixWorldInverse.copy(this.matrixWorld).invert()}updateWorldMatrix(e,t){super.updateWorldMatrix(e,t),this.matrixWorldInverse.copy(this.matrixWorld).invert()}clone(){return new this.constructor().copy(this)}},Gi=new D,H_=new ze,G_=new ze,Yt=class extends _c{constructor(e=50,t=1,i=.1,r=2e3){super(),this.isPerspectiveCamera=!0,this.type="PerspectiveCamera",this.fov=e,this.zoom=1,this.near=i,this.far=r,this.focus=10,this.aspect=t,this.view=null,this.filmGauge=35,this.filmOffset=0,this.updateProjectionMatrix()}copy(e,t){return super.copy(e,t),this.fov=e.fov,this.zoom=e.zoom,this.near=e.near,this.far=e.far,this.focus=e.focus,this.aspect=e.aspect,this.view=e.view===null?null:Object.assign({},e.view),this.filmGauge=e.filmGauge,this.filmOffset=e.filmOffset,this}setFocalLength(e){let t=.5*this.getFilmHeight()/e;this.fov=Df*2*Math.atan(t),this.updateProjectionMatrix()}getFocalLength(){let e=Math.tan(Ad*.5*this.fov);return .5*this.getFilmHeight()/e}getEffectiveFOV(){return Df*2*Math.atan(Math.tan(Ad*.5*this.fov)/this.zoom)}getFilmWidth(){return this.filmGauge*Math.min(this.aspect,1)}getFilmHeight(){return this.filmGauge/Math.max(this.aspect,1)}getViewBounds(e,t,i){Gi.set(-1,-1,.5).applyMatrix4(this.projectionMatrixInverse),t.set(Gi.x,Gi.y).multiplyScalar(-e/Gi.z),Gi.set(1,1,.5).applyMatrix4(this.projectionMatrixInverse),i.set(Gi.x,Gi.y).multiplyScalar(-e/Gi.z)}getViewSize(e,t){return this.getViewBounds(e,H_,G_),t.subVectors(G_,H_)}setViewOffset(e,t,i,r,s,o){this.aspect=e/t,this.view===null&&(this.view={enabled:!0,fullWidth:1,fullHeight:1,offsetX:0,offsetY:0,width:1,height:1}),this.view.enabled=!0,this.view.fullWidth=e,this.view.fullHeight=t,this.view.offsetX=i,this.view.offsetY=r,this.view.width=s,this.view.height=o,this.updateProjectionMatrix()}clearViewOffset(){this.view!==null&&(this.view.enabled=!1),this.updateProjectionMatrix()}updateProjectionMatrix(){let e=this.near,t=e*Math.tan(Ad*.5*this.fov)/this.zoom,i=2*t,r=this.aspect*i,s=-.5*r,o=this.view;if(this.view!==null&&this.view.enabled){let l=o.fullWidth,c=o.fullHeight;s+=o.offsetX*r/l,t-=o.offsetY*i/c,r*=o.width/l,i*=o.height/c}let a=this.filmOffset;a!==0&&(s+=e*a/this.getFilmWidth()),this.projectionMatrix.makePerspective(s,s+r,t,t-i,e,this.far,this.coordinateSystem),this.projectionMatrixInverse.copy(this.projectionMatrix).invert()}toJSON(e){let t=super.toJSON(e);return t.object.fov=this.fov,t.object.zoom=this.zoom,t.object.near=this.near,t.object.far=this.far,t.object.focus=this.focus,t.object.aspect=this.aspect,this.view!==null&&(t.object.view=Object.assign({},this.view)),t.object.filmGauge=this.filmGauge,t.object.filmOffset=this.filmOffset,t}},As=-90,Rs=1,zf=class extends Vt{constructor(e,t,i){super(),this.type="CubeCamera",this.renderTarget=i,this.coordinateSystem=null,this.activeMipmapLevel=0;let r=new Yt(As,Rs,e,t);r.layers=this.layers,this.add(r);let s=new Yt(As,Rs,e,t);s.layers=this.layers,this.add(s);let o=new Yt(As,Rs,e,t);o.layers=this.layers,this.add(o);let a=new Yt(As,Rs,e,t);a.layers=this.layers,this.add(a);let l=new Yt(As,Rs,e,t);l.layers=this.layers,this.add(l);let c=new Yt(As,Rs,e,t);c.layers=this.layers,this.add(c)}updateCoordinateSystem(){let e=this.coordinateSystem,t=this.children.concat(),[i,r,s,o,a,l]=t;for(let c of t)this.remove(c);if(e===ui)i.up.set(0,1,0),i.lookAt(1,0,0),r.up.set(0,1,0),r.lookAt(-1,0,0),s.up.set(0,0,-1),s.lookAt(0,1,0),o.up.set(0,0,1),o.lookAt(0,-1,0),a.up.set(0,1,0),a.lookAt(0,0,1),l.up.set(0,1,0),l.lookAt(0,0,-1);else if(e===hc)i.up.set(0,-1,0),i.lookAt(-1,0,0),r.up.set(0,-1,0),r.lookAt(1,0,0),s.up.set(0,0,1),s.lookAt(0,1,0),o.up.set(0,0,-1),o.lookAt(0,-1,0),a.up.set(0,-1,0),a.lookAt(0,0,1),l.up.set(0,-1,0),l.lookAt(0,0,-1);else throw new Error("THREE.CubeCamera.updateCoordinateSystem(): Invalid coordinate system: "+e);for(let c of t)this.add(c),c.updateMatrixWorld()}update(e,t){this.parent===null&&this.updateMatrixWorld();let{renderTarget:i,activeMipmapLevel:r}=this;this.coordinateSystem!==e.coordinateSystem&&(this.coordinateSystem=e.coordinateSystem,this.updateCoordinateSystem());let[s,o,a,l,c,u]=this.children,p=e.getRenderTarget(),f=e.getActiveCubeFace(),m=e.getActiveMipmapLevel(),_=e.xr.enabled;e.xr.enabled=!1;let y=i.texture.generateMipmaps;i.texture.generateMipmaps=!1,e.setRenderTarget(i,0,r),e.render(t,s),e.setRenderTarget(i,1,r),e.render(t,o),e.setRenderTarget(i,2,r),e.render(t,a),e.setRenderTarget(i,3,r),e.render(t,l),e.setRenderTarget(i,4,r),e.render(t,c),i.texture.generateMipmaps=y,e.setRenderTarget(i,5,r),e.render(t,u),e.setRenderTarget(p,f,m),e.xr.enabled=_,i.texture.needsPMREMUpdate=!0}},vc=class extends $t{constructor(e,t,i,r,s,o,a,l,c,u){e=e!==void 0?e:[],t=t!==void 0?t:Fs,super(e,t,i,r,s,o,a,l,c,u),this.isCubeTexture=!0,this.flipY=!1}get images(){return this.image}set images(e){this.image=e}},Vf=class extends fi{constructor(e=1,t={}){super(e,e,t),this.isWebGLCubeRenderTarget=!0;let i={width:e,height:e,depth:1},r=[i,i,i,i,i,i];this.texture=new vc(r,t.mapping,t.wrapS,t.wrapT,t.magFilter,t.minFilter,t.format,t.type,t.anisotropy,t.colorSpace),this.texture.isRenderTargetTexture=!0,this.texture.generateMipmaps=t.generateMipmaps!==void 0?t.generateMipmaps:!1,this.texture.minFilter=t.minFilter!==void 0?t.minFilter:qt}fromEquirectangularTexture(e,t){this.texture.type=t.type,this.texture.colorSpace=t.colorSpace,this.texture.generateMipmaps=t.generateMipmaps,this.texture.minFilter=t.minFilter,this.texture.magFilter=t.magFilter;let i={uniforms:{tEquirect:{value:null}},vertexShader:`

				varying vec3 vWorldDirection;

				vec3 transformDirection( in vec3 dir, in mat4 matrix ) {

					return normalize( ( matrix * vec4( dir, 0.0 ) ).xyz );

				}

				void main() {

					vWorldDirection = transformDirection( position, modelMatrix );

					#include <begin_vertex>
					#include <project_vertex>

				}
			`,fragmentShader:`

				uniform sampler2D tEquirect;

				varying vec3 vWorldDirection;

				#include <common>

				void main() {

					vec3 direction = normalize( vWorldDirection );

					vec2 sampleUV = equirectUv( direction );

					gl_FragColor = texture2D( tEquirect, sampleUV );

				}
			`},r=new Pr(5,5,5),s=new Mn({name:"CubemapFromEquirect",uniforms:Vs(i.uniforms),vertexShader:i.vertexShader,fragmentShader:i.fragmentShader,side:tn,blending:Yi});s.uniforms.tEquirect.value=t;let o=new St(r,s),a=t.minFilter;return t.minFilter===Cr&&(t.minFilter=qt),new zf(1,10,this).update(e,o),t.minFilter=a,o.geometry.dispose(),o.material.dispose(),this}clear(e,t,i,r){let s=e.getRenderTarget();for(let o=0;o<6;o++)e.setRenderTarget(this,o),e.clear(t,i,r);e.setRenderTarget(s)}},qd=new D,WS=new D,XS=new Ue,Ln=class{constructor(e=new D(1,0,0),t=0){this.isPlane=!0,this.normal=e,this.constant=t}set(e,t){return this.normal.copy(e),this.constant=t,this}setComponents(e,t,i,r){return this.normal.set(e,t,i),this.constant=r,this}setFromNormalAndCoplanarPoint(e,t){return this.normal.copy(e),this.constant=-t.dot(this.normal),this}setFromCoplanarPoints(e,t,i){let r=qd.subVectors(i,t).cross(WS.subVectors(e,t)).normalize();return this.setFromNormalAndCoplanarPoint(r,e),this}copy(e){return this.normal.copy(e.normal),this.constant=e.constant,this}normalize(){let e=1/this.normal.length();return this.normal.multiplyScalar(e),this.constant*=e,this}negate(){return this.constant*=-1,this.normal.negate(),this}distanceToPoint(e){return this.normal.dot(e)+this.constant}distanceToSphere(e){return this.distanceToPoint(e.center)-e.radius}projectPoint(e,t){return t.copy(e).addScaledVector(this.normal,-this.distanceToPoint(e))}intersectLine(e,t){let i=e.delta(qd),r=this.normal.dot(i);if(r===0)return this.distanceToPoint(e.start)===0?t.copy(e.start):null;let s=-(e.start.dot(this.normal)+this.constant)/r;return s<0||s>1?null:t.copy(e.start).addScaledVector(i,s)}intersectsLine(e){let t=this.distanceToPoint(e.start),i=this.distanceToPoint(e.end);return t<0&&i>0||i<0&&t>0}intersectsBox(e){return e.intersectsPlane(this)}intersectsSphere(e){return e.intersectsPlane(this)}coplanarPoint(e){return e.copy(this.normal).multiplyScalar(-this.constant)}applyMatrix4(e,t){let i=t||XS.getNormalMatrix(e),r=this.coplanarPoint(qd).applyMatrix4(e),s=this.normal.applyMatrix3(i).normalize();return this.constant=-r.dot(s),this}translate(e){return this.constant-=e.dot(this.normal),this}equals(e){return e.normal.equals(this.normal)&&e.constant===this.constant}clone(){return new this.constructor().copy(this)}},vr=new ia,Kl=new D,ra=class{constructor(e=new Ln,t=new Ln,i=new Ln,r=new Ln,s=new Ln,o=new Ln){this.planes=[e,t,i,r,s,o]}set(e,t,i,r,s,o){let a=this.planes;return a[0].copy(e),a[1].copy(t),a[2].copy(i),a[3].copy(r),a[4].copy(s),a[5].copy(o),this}copy(e){let t=this.planes;for(let i=0;i<6;i++)t[i].copy(e.planes[i]);return this}setFromProjectionMatrix(e,t=ui){let i=this.planes,r=e.elements,s=r[0],o=r[1],a=r[2],l=r[3],c=r[4],u=r[5],p=r[6],f=r[7],m=r[8],_=r[9],y=r[10],d=r[11],h=r[12],g=r[13],v=r[14],x=r[15];if(i[0].setComponents(l-s,f-c,d-m,x-h).normalize(),i[1].setComponents(l+s,f+c,d+m,x+h).normalize(),i[2].setComponents(l+o,f+u,d+_,x+g).normalize(),i[3].setComponents(l-o,f-u,d-_,x-g).normalize(),i[4].setComponents(l-a,f-p,d-y,x-v).normalize(),t===ui)i[5].setComponents(l+a,f+p,d+y,x+v).normalize();else if(t===hc)i[5].setComponents(a,p,y,v).normalize();else throw new Error("THREE.Frustum.setFromProjectionMatrix(): Invalid coordinate system: "+t);return this}intersectsObject(e){if(e.boundingSphere!==void 0)e.boundingSphere===null&&e.computeBoundingSphere(),vr.copy(e.boundingSphere).applyMatrix4(e.matrixWorld);else{let t=e.geometry;t.boundingSphere===null&&t.computeBoundingSphere(),vr.copy(t.boundingSphere).applyMatrix4(e.matrixWorld)}return this.intersectsSphere(vr)}intersectsSprite(e){return vr.center.set(0,0,0),vr.radius=.7071067811865476,vr.applyMatrix4(e.matrixWorld),this.intersectsSphere(vr)}intersectsSphere(e){let t=this.planes,i=e.center,r=-e.radius;for(let s=0;s<6;s++)if(t[s].distanceToPoint(i)<r)return!1;return!0}intersectsBox(e){let t=this.planes;for(let i=0;i<6;i++){let r=t[i];if(Kl.x=r.normal.x>0?e.max.x:e.min.x,Kl.y=r.normal.y>0?e.max.y:e.min.y,Kl.z=r.normal.z>0?e.max.z:e.min.z,r.distanceToPoint(Kl)<0)return!1}return!0}containsPoint(e){let t=this.planes;for(let i=0;i<6;i++)if(t[i].distanceToPoint(e)<0)return!1;return!0}clone(){return new this.constructor().copy(this)}};function Nv(){let n=null,e=!1,t=null,i=null;function r(s,o){t(s,o),i=n.requestAnimationFrame(r)}return{start:function(){e!==!0&&t!==null&&(i=n.requestAnimationFrame(r),e=!0)},stop:function(){n.cancelAnimationFrame(i),e=!1},setAnimationLoop:function(s){t=s},setContext:function(s){n=s}}}function YS(n){let e=new WeakMap;function t(a,l){let c=a.array,u=a.usage,p=c.byteLength,f=n.createBuffer();n.bindBuffer(l,f),n.bufferData(l,c,u),a.onUploadCallback();let m;if(c instanceof Float32Array)m=n.FLOAT;else if(c instanceof Uint16Array)a.isFloat16BufferAttribute?m=n.HALF_FLOAT:m=n.UNSIGNED_SHORT;else if(c instanceof Int16Array)m=n.SHORT;else if(c instanceof Uint32Array)m=n.UNSIGNED_INT;else if(c instanceof Int32Array)m=n.INT;else if(c instanceof Int8Array)m=n.BYTE;else if(c instanceof Uint8Array)m=n.UNSIGNED_BYTE;else if(c instanceof Uint8ClampedArray)m=n.UNSIGNED_BYTE;else throw new Error("THREE.WebGLAttributes: Unsupported buffer data format: "+c);return{buffer:f,type:m,bytesPerElement:c.BYTES_PER_ELEMENT,version:a.version,size:p}}function i(a,l,c){let u=l.array,p=l._updateRange,f=l.updateRanges;if(n.bindBuffer(c,a),p.count===-1&&f.length===0&&n.bufferSubData(c,0,u),f.length!==0){for(let m=0,_=f.length;m<_;m++){let y=f[m];n.bufferSubData(c,y.start*u.BYTES_PER_ELEMENT,u,y.start,y.count)}l.clearUpdateRanges()}p.count!==-1&&(n.bufferSubData(c,p.offset*u.BYTES_PER_ELEMENT,u,p.offset,p.count),p.count=-1),l.onUploadCallback()}function r(a){return a.isInterleavedBufferAttribute&&(a=a.data),e.get(a)}function s(a){a.isInterleavedBufferAttribute&&(a=a.data);let l=e.get(a);l&&(n.deleteBuffer(l.buffer),e.delete(a))}function o(a,l){if(a.isGLBufferAttribute){let u=e.get(a);(!u||u.version<a.version)&&e.set(a,{buffer:a.buffer,type:a.type,bytesPerElement:a.elementSize,version:a.version});return}a.isInterleavedBufferAttribute&&(a=a.data);let c=e.get(a);if(c===void 0)e.set(a,t(a,l));else if(c.version<a.version){if(c.size!==a.array.byteLength)throw new Error("THREE.WebGLAttributes: The size of the buffer attribute's array buffer does not match the original size. Resizing buffer attributes is not supported.");i(c.buffer,a,l),c.version=a.version}}return{get:r,remove:s,update:o}}var Yn=class n extends pi{constructor(e=1,t=1,i=1,r=1){super(),this.type="PlaneGeometry",this.parameters={width:e,height:t,widthSegments:i,heightSegments:r};let s=e/2,o=t/2,a=Math.floor(i),l=Math.floor(r),c=a+1,u=l+1,p=e/a,f=t/l,m=[],_=[],y=[],d=[];for(let h=0;h<u;h++){let g=h*f-o;for(let v=0;v<c;v++){let x=v*p-s;_.push(x,-g,0),y.push(0,0,1),d.push(v/a),d.push(1-h/l)}}for(let h=0;h<l;h++)for(let g=0;g<a;g++){let v=g+c*h,x=g+c*(h+1),R=g+1+c*(h+1),C=g+1+c*h;m.push(v,x,C),m.push(x,R,C)}this.setIndex(m),this.setAttribute("position",new hi(_,3)),this.setAttribute("normal",new hi(y,3)),this.setAttribute("uv",new hi(d,2))}copy(e){return super.copy(e),this.parameters=Object.assign({},e.parameters),this}static fromJSON(e){return new n(e.width,e.height,e.widthSegments,e.heightSegments)}},qS=`#ifdef USE_ALPHAHASH
	if ( diffuseColor.a < getAlphaHashThreshold( vPosition ) ) discard;
#endif`,$S=`#ifdef USE_ALPHAHASH
	const float ALPHA_HASH_SCALE = 0.05;
	float hash2D( vec2 value ) {
		return fract( 1.0e4 * sin( 17.0 * value.x + 0.1 * value.y ) * ( 0.1 + abs( sin( 13.0 * value.y + value.x ) ) ) );
	}
	float hash3D( vec3 value ) {
		return hash2D( vec2( hash2D( value.xy ), value.z ) );
	}
	float getAlphaHashThreshold( vec3 position ) {
		float maxDeriv = max(
			length( dFdx( position.xyz ) ),
			length( dFdy( position.xyz ) )
		);
		float pixScale = 1.0 / ( ALPHA_HASH_SCALE * maxDeriv );
		vec2 pixScales = vec2(
			exp2( floor( log2( pixScale ) ) ),
			exp2( ceil( log2( pixScale ) ) )
		);
		vec2 alpha = vec2(
			hash3D( floor( pixScales.x * position.xyz ) ),
			hash3D( floor( pixScales.y * position.xyz ) )
		);
		float lerpFactor = fract( log2( pixScale ) );
		float x = ( 1.0 - lerpFactor ) * alpha.x + lerpFactor * alpha.y;
		float a = min( lerpFactor, 1.0 - lerpFactor );
		vec3 cases = vec3(
			x * x / ( 2.0 * a * ( 1.0 - a ) ),
			( x - 0.5 * a ) / ( 1.0 - a ),
			1.0 - ( ( 1.0 - x ) * ( 1.0 - x ) / ( 2.0 * a * ( 1.0 - a ) ) )
		);
		float threshold = ( x < ( 1.0 - a ) )
			? ( ( x < a ) ? cases.x : cases.y )
			: cases.z;
		return clamp( threshold , 1.0e-6, 1.0 );
	}
#endif`,ZS=`#ifdef USE_ALPHAMAP
	diffuseColor.a *= texture2D( alphaMap, vAlphaMapUv ).g;
#endif`,JS=`#ifdef USE_ALPHAMAP
	uniform sampler2D alphaMap;
#endif`,KS=`#ifdef USE_ALPHATEST
	#ifdef ALPHA_TO_COVERAGE
	diffuseColor.a = smoothstep( alphaTest, alphaTest + fwidth( diffuseColor.a ), diffuseColor.a );
	if ( diffuseColor.a == 0.0 ) discard;
	#else
	if ( diffuseColor.a < alphaTest ) discard;
	#endif
#endif`,jS=`#ifdef USE_ALPHATEST
	uniform float alphaTest;
#endif`,QS=`#ifdef USE_AOMAP
	float ambientOcclusion = ( texture2D( aoMap, vAoMapUv ).r - 1.0 ) * aoMapIntensity + 1.0;
	reflectedLight.indirectDiffuse *= ambientOcclusion;
	#if defined( USE_CLEARCOAT ) 
		clearcoatSpecularIndirect *= ambientOcclusion;
	#endif
	#if defined( USE_SHEEN ) 
		sheenSpecularIndirect *= ambientOcclusion;
	#endif
	#if defined( USE_ENVMAP ) && defined( STANDARD )
		float dotNV = saturate( dot( geometryNormal, geometryViewDir ) );
		reflectedLight.indirectSpecular *= computeSpecularOcclusion( dotNV, ambientOcclusion, material.roughness );
	#endif
#endif`,e1=`#ifdef USE_AOMAP
	uniform sampler2D aoMap;
	uniform float aoMapIntensity;
#endif`,t1=`#ifdef USE_BATCHING
	#if ! defined( GL_ANGLE_multi_draw )
	#define gl_DrawID _gl_DrawID
	uniform int _gl_DrawID;
	#endif
	uniform highp sampler2D batchingTexture;
	uniform highp usampler2D batchingIdTexture;
	mat4 getBatchingMatrix( const in float i ) {
		int size = textureSize( batchingTexture, 0 ).x;
		int j = int( i ) * 4;
		int x = j % size;
		int y = j / size;
		vec4 v1 = texelFetch( batchingTexture, ivec2( x, y ), 0 );
		vec4 v2 = texelFetch( batchingTexture, ivec2( x + 1, y ), 0 );
		vec4 v3 = texelFetch( batchingTexture, ivec2( x + 2, y ), 0 );
		vec4 v4 = texelFetch( batchingTexture, ivec2( x + 3, y ), 0 );
		return mat4( v1, v2, v3, v4 );
	}
	float getIndirectIndex( const in int i ) {
		int size = textureSize( batchingIdTexture, 0 ).x;
		int x = i % size;
		int y = i / size;
		return float( texelFetch( batchingIdTexture, ivec2( x, y ), 0 ).r );
	}
#endif
#ifdef USE_BATCHING_COLOR
	uniform sampler2D batchingColorTexture;
	vec3 getBatchingColor( const in float i ) {
		int size = textureSize( batchingColorTexture, 0 ).x;
		int j = int( i );
		int x = j % size;
		int y = j / size;
		return texelFetch( batchingColorTexture, ivec2( x, y ), 0 ).rgb;
	}
#endif`,n1=`#ifdef USE_BATCHING
	mat4 batchingMatrix = getBatchingMatrix( getIndirectIndex( gl_DrawID ) );
#endif`,i1=`vec3 transformed = vec3( position );
#ifdef USE_ALPHAHASH
	vPosition = vec3( position );
#endif`,r1=`vec3 objectNormal = vec3( normal );
#ifdef USE_TANGENT
	vec3 objectTangent = vec3( tangent.xyz );
#endif`,s1=`float G_BlinnPhong_Implicit( ) {
	return 0.25;
}
float D_BlinnPhong( const in float shininess, const in float dotNH ) {
	return RECIPROCAL_PI * ( shininess * 0.5 + 1.0 ) * pow( dotNH, shininess );
}
vec3 BRDF_BlinnPhong( const in vec3 lightDir, const in vec3 viewDir, const in vec3 normal, const in vec3 specularColor, const in float shininess ) {
	vec3 halfDir = normalize( lightDir + viewDir );
	float dotNH = saturate( dot( normal, halfDir ) );
	float dotVH = saturate( dot( viewDir, halfDir ) );
	vec3 F = F_Schlick( specularColor, 1.0, dotVH );
	float G = G_BlinnPhong_Implicit( );
	float D = D_BlinnPhong( shininess, dotNH );
	return F * ( G * D );
} // validated`,o1=`#ifdef USE_IRIDESCENCE
	const mat3 XYZ_TO_REC709 = mat3(
		 3.2404542, -0.9692660,  0.0556434,
		-1.5371385,  1.8760108, -0.2040259,
		-0.4985314,  0.0415560,  1.0572252
	);
	vec3 Fresnel0ToIor( vec3 fresnel0 ) {
		vec3 sqrtF0 = sqrt( fresnel0 );
		return ( vec3( 1.0 ) + sqrtF0 ) / ( vec3( 1.0 ) - sqrtF0 );
	}
	vec3 IorToFresnel0( vec3 transmittedIor, float incidentIor ) {
		return pow2( ( transmittedIor - vec3( incidentIor ) ) / ( transmittedIor + vec3( incidentIor ) ) );
	}
	float IorToFresnel0( float transmittedIor, float incidentIor ) {
		return pow2( ( transmittedIor - incidentIor ) / ( transmittedIor + incidentIor ));
	}
	vec3 evalSensitivity( float OPD, vec3 shift ) {
		float phase = 2.0 * PI * OPD * 1.0e-9;
		vec3 val = vec3( 5.4856e-13, 4.4201e-13, 5.2481e-13 );
		vec3 pos = vec3( 1.6810e+06, 1.7953e+06, 2.2084e+06 );
		vec3 var = vec3( 4.3278e+09, 9.3046e+09, 6.6121e+09 );
		vec3 xyz = val * sqrt( 2.0 * PI * var ) * cos( pos * phase + shift ) * exp( - pow2( phase ) * var );
		xyz.x += 9.7470e-14 * sqrt( 2.0 * PI * 4.5282e+09 ) * cos( 2.2399e+06 * phase + shift[ 0 ] ) * exp( - 4.5282e+09 * pow2( phase ) );
		xyz /= 1.0685e-7;
		vec3 rgb = XYZ_TO_REC709 * xyz;
		return rgb;
	}
	vec3 evalIridescence( float outsideIOR, float eta2, float cosTheta1, float thinFilmThickness, vec3 baseF0 ) {
		vec3 I;
		float iridescenceIOR = mix( outsideIOR, eta2, smoothstep( 0.0, 0.03, thinFilmThickness ) );
		float sinTheta2Sq = pow2( outsideIOR / iridescenceIOR ) * ( 1.0 - pow2( cosTheta1 ) );
		float cosTheta2Sq = 1.0 - sinTheta2Sq;
		if ( cosTheta2Sq < 0.0 ) {
			return vec3( 1.0 );
		}
		float cosTheta2 = sqrt( cosTheta2Sq );
		float R0 = IorToFresnel0( iridescenceIOR, outsideIOR );
		float R12 = F_Schlick( R0, 1.0, cosTheta1 );
		float T121 = 1.0 - R12;
		float phi12 = 0.0;
		if ( iridescenceIOR < outsideIOR ) phi12 = PI;
		float phi21 = PI - phi12;
		vec3 baseIOR = Fresnel0ToIor( clamp( baseF0, 0.0, 0.9999 ) );		vec3 R1 = IorToFresnel0( baseIOR, iridescenceIOR );
		vec3 R23 = F_Schlick( R1, 1.0, cosTheta2 );
		vec3 phi23 = vec3( 0.0 );
		if ( baseIOR[ 0 ] < iridescenceIOR ) phi23[ 0 ] = PI;
		if ( baseIOR[ 1 ] < iridescenceIOR ) phi23[ 1 ] = PI;
		if ( baseIOR[ 2 ] < iridescenceIOR ) phi23[ 2 ] = PI;
		float OPD = 2.0 * iridescenceIOR * thinFilmThickness * cosTheta2;
		vec3 phi = vec3( phi21 ) + phi23;
		vec3 R123 = clamp( R12 * R23, 1e-5, 0.9999 );
		vec3 r123 = sqrt( R123 );
		vec3 Rs = pow2( T121 ) * R23 / ( vec3( 1.0 ) - R123 );
		vec3 C0 = R12 + Rs;
		I = C0;
		vec3 Cm = Rs - T121;
		for ( int m = 1; m <= 2; ++ m ) {
			Cm *= r123;
			vec3 Sm = 2.0 * evalSensitivity( float( m ) * OPD, float( m ) * phi );
			I += Cm * Sm;
		}
		return max( I, vec3( 0.0 ) );
	}
#endif`,a1=`#ifdef USE_BUMPMAP
	uniform sampler2D bumpMap;
	uniform float bumpScale;
	vec2 dHdxy_fwd() {
		vec2 dSTdx = dFdx( vBumpMapUv );
		vec2 dSTdy = dFdy( vBumpMapUv );
		float Hll = bumpScale * texture2D( bumpMap, vBumpMapUv ).x;
		float dBx = bumpScale * texture2D( bumpMap, vBumpMapUv + dSTdx ).x - Hll;
		float dBy = bumpScale * texture2D( bumpMap, vBumpMapUv + dSTdy ).x - Hll;
		return vec2( dBx, dBy );
	}
	vec3 perturbNormalArb( vec3 surf_pos, vec3 surf_norm, vec2 dHdxy, float faceDirection ) {
		vec3 vSigmaX = normalize( dFdx( surf_pos.xyz ) );
		vec3 vSigmaY = normalize( dFdy( surf_pos.xyz ) );
		vec3 vN = surf_norm;
		vec3 R1 = cross( vSigmaY, vN );
		vec3 R2 = cross( vN, vSigmaX );
		float fDet = dot( vSigmaX, R1 ) * faceDirection;
		vec3 vGrad = sign( fDet ) * ( dHdxy.x * R1 + dHdxy.y * R2 );
		return normalize( abs( fDet ) * surf_norm - vGrad );
	}
#endif`,l1=`#if NUM_CLIPPING_PLANES > 0
	vec4 plane;
	#ifdef ALPHA_TO_COVERAGE
		float distanceToPlane, distanceGradient;
		float clipOpacity = 1.0;
		#pragma unroll_loop_start
		for ( int i = 0; i < UNION_CLIPPING_PLANES; i ++ ) {
			plane = clippingPlanes[ i ];
			distanceToPlane = - dot( vClipPosition, plane.xyz ) + plane.w;
			distanceGradient = fwidth( distanceToPlane ) / 2.0;
			clipOpacity *= smoothstep( - distanceGradient, distanceGradient, distanceToPlane );
			if ( clipOpacity == 0.0 ) discard;
		}
		#pragma unroll_loop_end
		#if UNION_CLIPPING_PLANES < NUM_CLIPPING_PLANES
			float unionClipOpacity = 1.0;
			#pragma unroll_loop_start
			for ( int i = UNION_CLIPPING_PLANES; i < NUM_CLIPPING_PLANES; i ++ ) {
				plane = clippingPlanes[ i ];
				distanceToPlane = - dot( vClipPosition, plane.xyz ) + plane.w;
				distanceGradient = fwidth( distanceToPlane ) / 2.0;
				unionClipOpacity *= 1.0 - smoothstep( - distanceGradient, distanceGradient, distanceToPlane );
			}
			#pragma unroll_loop_end
			clipOpacity *= 1.0 - unionClipOpacity;
		#endif
		diffuseColor.a *= clipOpacity;
		if ( diffuseColor.a == 0.0 ) discard;
	#else
		#pragma unroll_loop_start
		for ( int i = 0; i < UNION_CLIPPING_PLANES; i ++ ) {
			plane = clippingPlanes[ i ];
			if ( dot( vClipPosition, plane.xyz ) > plane.w ) discard;
		}
		#pragma unroll_loop_end
		#if UNION_CLIPPING_PLANES < NUM_CLIPPING_PLANES
			bool clipped = true;
			#pragma unroll_loop_start
			for ( int i = UNION_CLIPPING_PLANES; i < NUM_CLIPPING_PLANES; i ++ ) {
				plane = clippingPlanes[ i ];
				clipped = ( dot( vClipPosition, plane.xyz ) > plane.w ) && clipped;
			}
			#pragma unroll_loop_end
			if ( clipped ) discard;
		#endif
	#endif
#endif`,c1=`#if NUM_CLIPPING_PLANES > 0
	varying vec3 vClipPosition;
	uniform vec4 clippingPlanes[ NUM_CLIPPING_PLANES ];
#endif`,u1=`#if NUM_CLIPPING_PLANES > 0
	varying vec3 vClipPosition;
#endif`,h1=`#if NUM_CLIPPING_PLANES > 0
	vClipPosition = - mvPosition.xyz;
#endif`,d1=`#if defined( USE_COLOR_ALPHA )
	diffuseColor *= vColor;
#elif defined( USE_COLOR )
	diffuseColor.rgb *= vColor;
#endif`,f1=`#if defined( USE_COLOR_ALPHA )
	varying vec4 vColor;
#elif defined( USE_COLOR )
	varying vec3 vColor;
#endif`,p1=`#if defined( USE_COLOR_ALPHA )
	varying vec4 vColor;
#elif defined( USE_COLOR ) || defined( USE_INSTANCING_COLOR ) || defined( USE_BATCHING_COLOR )
	varying vec3 vColor;
#endif`,m1=`#if defined( USE_COLOR_ALPHA )
	vColor = vec4( 1.0 );
#elif defined( USE_COLOR ) || defined( USE_INSTANCING_COLOR ) || defined( USE_BATCHING_COLOR )
	vColor = vec3( 1.0 );
#endif
#ifdef USE_COLOR
	vColor *= color;
#endif
#ifdef USE_INSTANCING_COLOR
	vColor.xyz *= instanceColor.xyz;
#endif
#ifdef USE_BATCHING_COLOR
	vec3 batchingColor = getBatchingColor( getIndirectIndex( gl_DrawID ) );
	vColor.xyz *= batchingColor.xyz;
#endif`,g1=`#define PI 3.141592653589793
#define PI2 6.283185307179586
#define PI_HALF 1.5707963267948966
#define RECIPROCAL_PI 0.3183098861837907
#define RECIPROCAL_PI2 0.15915494309189535
#define EPSILON 1e-6
#ifndef saturate
#define saturate( a ) clamp( a, 0.0, 1.0 )
#endif
#define whiteComplement( a ) ( 1.0 - saturate( a ) )
float pow2( const in float x ) { return x*x; }
vec3 pow2( const in vec3 x ) { return x*x; }
float pow3( const in float x ) { return x*x*x; }
float pow4( const in float x ) { float x2 = x*x; return x2*x2; }
float max3( const in vec3 v ) { return max( max( v.x, v.y ), v.z ); }
float average( const in vec3 v ) { return dot( v, vec3( 0.3333333 ) ); }
highp float rand( const in vec2 uv ) {
	const highp float a = 12.9898, b = 78.233, c = 43758.5453;
	highp float dt = dot( uv.xy, vec2( a,b ) ), sn = mod( dt, PI );
	return fract( sin( sn ) * c );
}
#ifdef HIGH_PRECISION
	float precisionSafeLength( vec3 v ) { return length( v ); }
#else
	float precisionSafeLength( vec3 v ) {
		float maxComponent = max3( abs( v ) );
		return length( v / maxComponent ) * maxComponent;
	}
#endif
struct IncidentLight {
	vec3 color;
	vec3 direction;
	bool visible;
};
struct ReflectedLight {
	vec3 directDiffuse;
	vec3 directSpecular;
	vec3 indirectDiffuse;
	vec3 indirectSpecular;
};
#ifdef USE_ALPHAHASH
	varying vec3 vPosition;
#endif
vec3 transformDirection( in vec3 dir, in mat4 matrix ) {
	return normalize( ( matrix * vec4( dir, 0.0 ) ).xyz );
}
vec3 inverseTransformDirection( in vec3 dir, in mat4 matrix ) {
	return normalize( ( vec4( dir, 0.0 ) * matrix ).xyz );
}
mat3 transposeMat3( const in mat3 m ) {
	mat3 tmp;
	tmp[ 0 ] = vec3( m[ 0 ].x, m[ 1 ].x, m[ 2 ].x );
	tmp[ 1 ] = vec3( m[ 0 ].y, m[ 1 ].y, m[ 2 ].y );
	tmp[ 2 ] = vec3( m[ 0 ].z, m[ 1 ].z, m[ 2 ].z );
	return tmp;
}
float luminance( const in vec3 rgb ) {
	const vec3 weights = vec3( 0.2126729, 0.7151522, 0.0721750 );
	return dot( weights, rgb );
}
bool isPerspectiveMatrix( mat4 m ) {
	return m[ 2 ][ 3 ] == - 1.0;
}
vec2 equirectUv( in vec3 dir ) {
	float u = atan( dir.z, dir.x ) * RECIPROCAL_PI2 + 0.5;
	float v = asin( clamp( dir.y, - 1.0, 1.0 ) ) * RECIPROCAL_PI + 0.5;
	return vec2( u, v );
}
vec3 BRDF_Lambert( const in vec3 diffuseColor ) {
	return RECIPROCAL_PI * diffuseColor;
}
vec3 F_Schlick( const in vec3 f0, const in float f90, const in float dotVH ) {
	float fresnel = exp2( ( - 5.55473 * dotVH - 6.98316 ) * dotVH );
	return f0 * ( 1.0 - fresnel ) + ( f90 * fresnel );
}
float F_Schlick( const in float f0, const in float f90, const in float dotVH ) {
	float fresnel = exp2( ( - 5.55473 * dotVH - 6.98316 ) * dotVH );
	return f0 * ( 1.0 - fresnel ) + ( f90 * fresnel );
} // validated`,_1=`#ifdef ENVMAP_TYPE_CUBE_UV
	#define cubeUV_minMipLevel 4.0
	#define cubeUV_minTileSize 16.0
	float getFace( vec3 direction ) {
		vec3 absDirection = abs( direction );
		float face = - 1.0;
		if ( absDirection.x > absDirection.z ) {
			if ( absDirection.x > absDirection.y )
				face = direction.x > 0.0 ? 0.0 : 3.0;
			else
				face = direction.y > 0.0 ? 1.0 : 4.0;
		} else {
			if ( absDirection.z > absDirection.y )
				face = direction.z > 0.0 ? 2.0 : 5.0;
			else
				face = direction.y > 0.0 ? 1.0 : 4.0;
		}
		return face;
	}
	vec2 getUV( vec3 direction, float face ) {
		vec2 uv;
		if ( face == 0.0 ) {
			uv = vec2( direction.z, direction.y ) / abs( direction.x );
		} else if ( face == 1.0 ) {
			uv = vec2( - direction.x, - direction.z ) / abs( direction.y );
		} else if ( face == 2.0 ) {
			uv = vec2( - direction.x, direction.y ) / abs( direction.z );
		} else if ( face == 3.0 ) {
			uv = vec2( - direction.z, direction.y ) / abs( direction.x );
		} else if ( face == 4.0 ) {
			uv = vec2( - direction.x, direction.z ) / abs( direction.y );
		} else {
			uv = vec2( direction.x, direction.y ) / abs( direction.z );
		}
		return 0.5 * ( uv + 1.0 );
	}
	vec3 bilinearCubeUV( sampler2D envMap, vec3 direction, float mipInt ) {
		float face = getFace( direction );
		float filterInt = max( cubeUV_minMipLevel - mipInt, 0.0 );
		mipInt = max( mipInt, cubeUV_minMipLevel );
		float faceSize = exp2( mipInt );
		highp vec2 uv = getUV( direction, face ) * ( faceSize - 2.0 ) + 1.0;
		if ( face > 2.0 ) {
			uv.y += faceSize;
			face -= 3.0;
		}
		uv.x += face * faceSize;
		uv.x += filterInt * 3.0 * cubeUV_minTileSize;
		uv.y += 4.0 * ( exp2( CUBEUV_MAX_MIP ) - faceSize );
		uv.x *= CUBEUV_TEXEL_WIDTH;
		uv.y *= CUBEUV_TEXEL_HEIGHT;
		#ifdef texture2DGradEXT
			return texture2DGradEXT( envMap, uv, vec2( 0.0 ), vec2( 0.0 ) ).rgb;
		#else
			return texture2D( envMap, uv ).rgb;
		#endif
	}
	#define cubeUV_r0 1.0
	#define cubeUV_m0 - 2.0
	#define cubeUV_r1 0.8
	#define cubeUV_m1 - 1.0
	#define cubeUV_r4 0.4
	#define cubeUV_m4 2.0
	#define cubeUV_r5 0.305
	#define cubeUV_m5 3.0
	#define cubeUV_r6 0.21
	#define cubeUV_m6 4.0
	float roughnessToMip( float roughness ) {
		float mip = 0.0;
		if ( roughness >= cubeUV_r1 ) {
			mip = ( cubeUV_r0 - roughness ) * ( cubeUV_m1 - cubeUV_m0 ) / ( cubeUV_r0 - cubeUV_r1 ) + cubeUV_m0;
		} else if ( roughness >= cubeUV_r4 ) {
			mip = ( cubeUV_r1 - roughness ) * ( cubeUV_m4 - cubeUV_m1 ) / ( cubeUV_r1 - cubeUV_r4 ) + cubeUV_m1;
		} else if ( roughness >= cubeUV_r5 ) {
			mip = ( cubeUV_r4 - roughness ) * ( cubeUV_m5 - cubeUV_m4 ) / ( cubeUV_r4 - cubeUV_r5 ) + cubeUV_m4;
		} else if ( roughness >= cubeUV_r6 ) {
			mip = ( cubeUV_r5 - roughness ) * ( cubeUV_m6 - cubeUV_m5 ) / ( cubeUV_r5 - cubeUV_r6 ) + cubeUV_m5;
		} else {
			mip = - 2.0 * log2( 1.16 * roughness );		}
		return mip;
	}
	vec4 textureCubeUV( sampler2D envMap, vec3 sampleDir, float roughness ) {
		float mip = clamp( roughnessToMip( roughness ), cubeUV_m0, CUBEUV_MAX_MIP );
		float mipF = fract( mip );
		float mipInt = floor( mip );
		vec3 color0 = bilinearCubeUV( envMap, sampleDir, mipInt );
		if ( mipF == 0.0 ) {
			return vec4( color0, 1.0 );
		} else {
			vec3 color1 = bilinearCubeUV( envMap, sampleDir, mipInt + 1.0 );
			return vec4( mix( color0, color1, mipF ), 1.0 );
		}
	}
#endif`,v1=`vec3 transformedNormal = objectNormal;
#ifdef USE_TANGENT
	vec3 transformedTangent = objectTangent;
#endif
#ifdef USE_BATCHING
	mat3 bm = mat3( batchingMatrix );
	transformedNormal /= vec3( dot( bm[ 0 ], bm[ 0 ] ), dot( bm[ 1 ], bm[ 1 ] ), dot( bm[ 2 ], bm[ 2 ] ) );
	transformedNormal = bm * transformedNormal;
	#ifdef USE_TANGENT
		transformedTangent = bm * transformedTangent;
	#endif
#endif
#ifdef USE_INSTANCING
	mat3 im = mat3( instanceMatrix );
	transformedNormal /= vec3( dot( im[ 0 ], im[ 0 ] ), dot( im[ 1 ], im[ 1 ] ), dot( im[ 2 ], im[ 2 ] ) );
	transformedNormal = im * transformedNormal;
	#ifdef USE_TANGENT
		transformedTangent = im * transformedTangent;
	#endif
#endif
transformedNormal = normalMatrix * transformedNormal;
#ifdef FLIP_SIDED
	transformedNormal = - transformedNormal;
#endif
#ifdef USE_TANGENT
	transformedTangent = ( modelViewMatrix * vec4( transformedTangent, 0.0 ) ).xyz;
	#ifdef FLIP_SIDED
		transformedTangent = - transformedTangent;
	#endif
#endif`,y1=`#ifdef USE_DISPLACEMENTMAP
	uniform sampler2D displacementMap;
	uniform float displacementScale;
	uniform float displacementBias;
#endif`,x1=`#ifdef USE_DISPLACEMENTMAP
	transformed += normalize( objectNormal ) * ( texture2D( displacementMap, vDisplacementMapUv ).x * displacementScale + displacementBias );
#endif`,w1=`#ifdef USE_EMISSIVEMAP
	vec4 emissiveColor = texture2D( emissiveMap, vEmissiveMapUv );
	totalEmissiveRadiance *= emissiveColor.rgb;
#endif`,S1=`#ifdef USE_EMISSIVEMAP
	uniform sampler2D emissiveMap;
#endif`,M1="gl_FragColor = linearToOutputTexel( gl_FragColor );",E1=`
const mat3 LINEAR_SRGB_TO_LINEAR_DISPLAY_P3 = mat3(
	vec3( 0.8224621, 0.177538, 0.0 ),
	vec3( 0.0331941, 0.9668058, 0.0 ),
	vec3( 0.0170827, 0.0723974, 0.9105199 )
);
const mat3 LINEAR_DISPLAY_P3_TO_LINEAR_SRGB = mat3(
	vec3( 1.2249401, - 0.2249404, 0.0 ),
	vec3( - 0.0420569, 1.0420571, 0.0 ),
	vec3( - 0.0196376, - 0.0786361, 1.0982735 )
);
vec4 LinearSRGBToLinearDisplayP3( in vec4 value ) {
	return vec4( value.rgb * LINEAR_SRGB_TO_LINEAR_DISPLAY_P3, value.a );
}
vec4 LinearDisplayP3ToLinearSRGB( in vec4 value ) {
	return vec4( value.rgb * LINEAR_DISPLAY_P3_TO_LINEAR_SRGB, value.a );
}
vec4 LinearTransferOETF( in vec4 value ) {
	return value;
}
vec4 sRGBTransferOETF( in vec4 value ) {
	return vec4( mix( pow( value.rgb, vec3( 0.41666 ) ) * 1.055 - vec3( 0.055 ), value.rgb * 12.92, vec3( lessThanEqual( value.rgb, vec3( 0.0031308 ) ) ) ), value.a );
}
vec4 LinearToLinear( in vec4 value ) {
	return value;
}
vec4 LinearTosRGB( in vec4 value ) {
	return sRGBTransferOETF( value );
}`,C1=`#ifdef USE_ENVMAP
	#ifdef ENV_WORLDPOS
		vec3 cameraToFrag;
		if ( isOrthographic ) {
			cameraToFrag = normalize( vec3( - viewMatrix[ 0 ][ 2 ], - viewMatrix[ 1 ][ 2 ], - viewMatrix[ 2 ][ 2 ] ) );
		} else {
			cameraToFrag = normalize( vWorldPosition - cameraPosition );
		}
		vec3 worldNormal = inverseTransformDirection( normal, viewMatrix );
		#ifdef ENVMAP_MODE_REFLECTION
			vec3 reflectVec = reflect( cameraToFrag, worldNormal );
		#else
			vec3 reflectVec = refract( cameraToFrag, worldNormal, refractionRatio );
		#endif
	#else
		vec3 reflectVec = vReflect;
	#endif
	#ifdef ENVMAP_TYPE_CUBE
		vec4 envColor = textureCube( envMap, envMapRotation * vec3( flipEnvMap * reflectVec.x, reflectVec.yz ) );
	#else
		vec4 envColor = vec4( 0.0 );
	#endif
	#ifdef ENVMAP_BLENDING_MULTIPLY
		outgoingLight = mix( outgoingLight, outgoingLight * envColor.xyz, specularStrength * reflectivity );
	#elif defined( ENVMAP_BLENDING_MIX )
		outgoingLight = mix( outgoingLight, envColor.xyz, specularStrength * reflectivity );
	#elif defined( ENVMAP_BLENDING_ADD )
		outgoingLight += envColor.xyz * specularStrength * reflectivity;
	#endif
#endif`,T1=`#ifdef USE_ENVMAP
	uniform float envMapIntensity;
	uniform float flipEnvMap;
	uniform mat3 envMapRotation;
	#ifdef ENVMAP_TYPE_CUBE
		uniform samplerCube envMap;
	#else
		uniform sampler2D envMap;
	#endif
	
#endif`,b1=`#ifdef USE_ENVMAP
	uniform float reflectivity;
	#if defined( USE_BUMPMAP ) || defined( USE_NORMALMAP ) || defined( PHONG ) || defined( LAMBERT )
		#define ENV_WORLDPOS
	#endif
	#ifdef ENV_WORLDPOS
		varying vec3 vWorldPosition;
		uniform float refractionRatio;
	#else
		varying vec3 vReflect;
	#endif
#endif`,A1=`#ifdef USE_ENVMAP
	#if defined( USE_BUMPMAP ) || defined( USE_NORMALMAP ) || defined( PHONG ) || defined( LAMBERT )
		#define ENV_WORLDPOS
	#endif
	#ifdef ENV_WORLDPOS
		
		varying vec3 vWorldPosition;
	#else
		varying vec3 vReflect;
		uniform float refractionRatio;
	#endif
#endif`,R1=`#ifdef USE_ENVMAP
	#ifdef ENV_WORLDPOS
		vWorldPosition = worldPosition.xyz;
	#else
		vec3 cameraToVertex;
		if ( isOrthographic ) {
			cameraToVertex = normalize( vec3( - viewMatrix[ 0 ][ 2 ], - viewMatrix[ 1 ][ 2 ], - viewMatrix[ 2 ][ 2 ] ) );
		} else {
			cameraToVertex = normalize( worldPosition.xyz - cameraPosition );
		}
		vec3 worldNormal = inverseTransformDirection( transformedNormal, viewMatrix );
		#ifdef ENVMAP_MODE_REFLECTION
			vReflect = reflect( cameraToVertex, worldNormal );
		#else
			vReflect = refract( cameraToVertex, worldNormal, refractionRatio );
		#endif
	#endif
#endif`,P1=`#ifdef USE_FOG
	vFogDepth = - mvPosition.z;
#endif`,I1=`#ifdef USE_FOG
	varying float vFogDepth;
#endif`,L1=`#ifdef USE_FOG
	#ifdef FOG_EXP2
		float fogFactor = 1.0 - exp( - fogDensity * fogDensity * vFogDepth * vFogDepth );
	#else
		float fogFactor = smoothstep( fogNear, fogFar, vFogDepth );
	#endif
	gl_FragColor.rgb = mix( gl_FragColor.rgb, fogColor, fogFactor );
#endif`,U1=`#ifdef USE_FOG
	uniform vec3 fogColor;
	varying float vFogDepth;
	#ifdef FOG_EXP2
		uniform float fogDensity;
	#else
		uniform float fogNear;
		uniform float fogFar;
	#endif
#endif`,N1=`#ifdef USE_GRADIENTMAP
	uniform sampler2D gradientMap;
#endif
vec3 getGradientIrradiance( vec3 normal, vec3 lightDirection ) {
	float dotNL = dot( normal, lightDirection );
	vec2 coord = vec2( dotNL * 0.5 + 0.5, 0.0 );
	#ifdef USE_GRADIENTMAP
		return vec3( texture2D( gradientMap, coord ).r );
	#else
		vec2 fw = fwidth( coord ) * 0.5;
		return mix( vec3( 0.7 ), vec3( 1.0 ), smoothstep( 0.7 - fw.x, 0.7 + fw.x, coord.x ) );
	#endif
}`,D1=`#ifdef USE_LIGHTMAP
	uniform sampler2D lightMap;
	uniform float lightMapIntensity;
#endif`,O1=`LambertMaterial material;
material.diffuseColor = diffuseColor.rgb;
material.specularStrength = specularStrength;`,F1=`varying vec3 vViewPosition;
struct LambertMaterial {
	vec3 diffuseColor;
	float specularStrength;
};
void RE_Direct_Lambert( const in IncidentLight directLight, const in vec3 geometryPosition, const in vec3 geometryNormal, const in vec3 geometryViewDir, const in vec3 geometryClearcoatNormal, const in LambertMaterial material, inout ReflectedLight reflectedLight ) {
	float dotNL = saturate( dot( geometryNormal, directLight.direction ) );
	vec3 irradiance = dotNL * directLight.color;
	reflectedLight.directDiffuse += irradiance * BRDF_Lambert( material.diffuseColor );
}
void RE_IndirectDiffuse_Lambert( const in vec3 irradiance, const in vec3 geometryPosition, const in vec3 geometryNormal, const in vec3 geometryViewDir, const in vec3 geometryClearcoatNormal, const in LambertMaterial material, inout ReflectedLight reflectedLight ) {
	reflectedLight.indirectDiffuse += irradiance * BRDF_Lambert( material.diffuseColor );
}
#define RE_Direct				RE_Direct_Lambert
#define RE_IndirectDiffuse		RE_IndirectDiffuse_Lambert`,B1=`uniform bool receiveShadow;
uniform vec3 ambientLightColor;
#if defined( USE_LIGHT_PROBES )
	uniform vec3 lightProbe[ 9 ];
#endif
vec3 shGetIrradianceAt( in vec3 normal, in vec3 shCoefficients[ 9 ] ) {
	float x = normal.x, y = normal.y, z = normal.z;
	vec3 result = shCoefficients[ 0 ] * 0.886227;
	result += shCoefficients[ 1 ] * 2.0 * 0.511664 * y;
	result += shCoefficients[ 2 ] * 2.0 * 0.511664 * z;
	result += shCoefficients[ 3 ] * 2.0 * 0.511664 * x;
	result += shCoefficients[ 4 ] * 2.0 * 0.429043 * x * y;
	result += shCoefficients[ 5 ] * 2.0 * 0.429043 * y * z;
	result += shCoefficients[ 6 ] * ( 0.743125 * z * z - 0.247708 );
	result += shCoefficients[ 7 ] * 2.0 * 0.429043 * x * z;
	result += shCoefficients[ 8 ] * 0.429043 * ( x * x - y * y );
	return result;
}
vec3 getLightProbeIrradiance( const in vec3 lightProbe[ 9 ], const in vec3 normal ) {
	vec3 worldNormal = inverseTransformDirection( normal, viewMatrix );
	vec3 irradiance = shGetIrradianceAt( worldNormal, lightProbe );
	return irradiance;
}
vec3 getAmbientLightIrradiance( const in vec3 ambientLightColor ) {
	vec3 irradiance = ambientLightColor;
	return irradiance;
}
float getDistanceAttenuation( const in float lightDistance, const in float cutoffDistance, const in float decayExponent ) {
	float distanceFalloff = 1.0 / max( pow( lightDistance, decayExponent ), 0.01 );
	if ( cutoffDistance > 0.0 ) {
		distanceFalloff *= pow2( saturate( 1.0 - pow4( lightDistance / cutoffDistance ) ) );
	}
	return distanceFalloff;
}
float getSpotAttenuation( const in float coneCosine, const in float penumbraCosine, const in float angleCosine ) {
	return smoothstep( coneCosine, penumbraCosine, angleCosine );
}
#if NUM_DIR_LIGHTS > 0
	struct DirectionalLight {
		vec3 direction;
		vec3 color;
	};
	uniform DirectionalLight directionalLights[ NUM_DIR_LIGHTS ];
	void getDirectionalLightInfo( const in DirectionalLight directionalLight, out IncidentLight light ) {
		light.color = directionalLight.color;
		light.direction = directionalLight.direction;
		light.visible = true;
	}
#endif
#if NUM_POINT_LIGHTS > 0
	struct PointLight {
		vec3 position;
		vec3 color;
		float distance;
		float decay;
	};
	uniform PointLight pointLights[ NUM_POINT_LIGHTS ];
	void getPointLightInfo( const in PointLight pointLight, const in vec3 geometryPosition, out IncidentLight light ) {
		vec3 lVector = pointLight.position - geometryPosition;
		light.direction = normalize( lVector );
		float lightDistance = length( lVector );
		light.color = pointLight.color;
		light.color *= getDistanceAttenuation( lightDistance, pointLight.distance, pointLight.decay );
		light.visible = ( light.color != vec3( 0.0 ) );
	}
#endif
#if NUM_SPOT_LIGHTS > 0
	struct SpotLight {
		vec3 position;
		vec3 direction;
		vec3 color;
		float distance;
		float decay;
		float coneCos;
		float penumbraCos;
	};
	uniform SpotLight spotLights[ NUM_SPOT_LIGHTS ];
	void getSpotLightInfo( const in SpotLight spotLight, const in vec3 geometryPosition, out IncidentLight light ) {
		vec3 lVector = spotLight.position - geometryPosition;
		light.direction = normalize( lVector );
		float angleCos = dot( light.direction, spotLight.direction );
		float spotAttenuation = getSpotAttenuation( spotLight.coneCos, spotLight.penumbraCos, angleCos );
		if ( spotAttenuation > 0.0 ) {
			float lightDistance = length( lVector );
			light.color = spotLight.color * spotAttenuation;
			light.color *= getDistanceAttenuation( lightDistance, spotLight.distance, spotLight.decay );
			light.visible = ( light.color != vec3( 0.0 ) );
		} else {
			light.color = vec3( 0.0 );
			light.visible = false;
		}
	}
#endif
#if NUM_RECT_AREA_LIGHTS > 0
	struct RectAreaLight {
		vec3 color;
		vec3 position;
		vec3 halfWidth;
		vec3 halfHeight;
	};
	uniform sampler2D ltc_1;	uniform sampler2D ltc_2;
	uniform RectAreaLight rectAreaLights[ NUM_RECT_AREA_LIGHTS ];
#endif
#if NUM_HEMI_LIGHTS > 0
	struct HemisphereLight {
		vec3 direction;
		vec3 skyColor;
		vec3 groundColor;
	};
	uniform HemisphereLight hemisphereLights[ NUM_HEMI_LIGHTS ];
	vec3 getHemisphereLightIrradiance( const in HemisphereLight hemiLight, const in vec3 normal ) {
		float dotNL = dot( normal, hemiLight.direction );
		float hemiDiffuseWeight = 0.5 * dotNL + 0.5;
		vec3 irradiance = mix( hemiLight.groundColor, hemiLight.skyColor, hemiDiffuseWeight );
		return irradiance;
	}
#endif`,k1=`#ifdef USE_ENVMAP
	vec3 getIBLIrradiance( const in vec3 normal ) {
		#ifdef ENVMAP_TYPE_CUBE_UV
			vec3 worldNormal = inverseTransformDirection( normal, viewMatrix );
			vec4 envMapColor = textureCubeUV( envMap, envMapRotation * worldNormal, 1.0 );
			return PI * envMapColor.rgb * envMapIntensity;
		#else
			return vec3( 0.0 );
		#endif
	}
	vec3 getIBLRadiance( const in vec3 viewDir, const in vec3 normal, const in float roughness ) {
		#ifdef ENVMAP_TYPE_CUBE_UV
			vec3 reflectVec = reflect( - viewDir, normal );
			reflectVec = normalize( mix( reflectVec, normal, roughness * roughness) );
			reflectVec = inverseTransformDirection( reflectVec, viewMatrix );
			vec4 envMapColor = textureCubeUV( envMap, envMapRotation * reflectVec, roughness );
			return envMapColor.rgb * envMapIntensity;
		#else
			return vec3( 0.0 );
		#endif
	}
	#ifdef USE_ANISOTROPY
		vec3 getIBLAnisotropyRadiance( const in vec3 viewDir, const in vec3 normal, const in float roughness, const in vec3 bitangent, const in float anisotropy ) {
			#ifdef ENVMAP_TYPE_CUBE_UV
				vec3 bentNormal = cross( bitangent, viewDir );
				bentNormal = normalize( cross( bentNormal, bitangent ) );
				bentNormal = normalize( mix( bentNormal, normal, pow2( pow2( 1.0 - anisotropy * ( 1.0 - roughness ) ) ) ) );
				return getIBLRadiance( viewDir, bentNormal, roughness );
			#else
				return vec3( 0.0 );
			#endif
		}
	#endif
#endif`,z1=`ToonMaterial material;
material.diffuseColor = diffuseColor.rgb;`,V1=`varying vec3 vViewPosition;
struct ToonMaterial {
	vec3 diffuseColor;
};
void RE_Direct_Toon( const in IncidentLight directLight, const in vec3 geometryPosition, const in vec3 geometryNormal, const in vec3 geometryViewDir, const in vec3 geometryClearcoatNormal, const in ToonMaterial material, inout ReflectedLight reflectedLight ) {
	vec3 irradiance = getGradientIrradiance( geometryNormal, directLight.direction ) * directLight.color;
	reflectedLight.directDiffuse += irradiance * BRDF_Lambert( material.diffuseColor );
}
void RE_IndirectDiffuse_Toon( const in vec3 irradiance, const in vec3 geometryPosition, const in vec3 geometryNormal, const in vec3 geometryViewDir, const in vec3 geometryClearcoatNormal, const in ToonMaterial material, inout ReflectedLight reflectedLight ) {
	reflectedLight.indirectDiffuse += irradiance * BRDF_Lambert( material.diffuseColor );
}
#define RE_Direct				RE_Direct_Toon
#define RE_IndirectDiffuse		RE_IndirectDiffuse_Toon`,H1=`BlinnPhongMaterial material;
material.diffuseColor = diffuseColor.rgb;
material.specularColor = specular;
material.specularShininess = shininess;
material.specularStrength = specularStrength;`,G1=`varying vec3 vViewPosition;
struct BlinnPhongMaterial {
	vec3 diffuseColor;
	vec3 specularColor;
	float specularShininess;
	float specularStrength;
};
void RE_Direct_BlinnPhong( const in IncidentLight directLight, const in vec3 geometryPosition, const in vec3 geometryNormal, const in vec3 geometryViewDir, const in vec3 geometryClearcoatNormal, const in BlinnPhongMaterial material, inout ReflectedLight reflectedLight ) {
	float dotNL = saturate( dot( geometryNormal, directLight.direction ) );
	vec3 irradiance = dotNL * directLight.color;
	reflectedLight.directDiffuse += irradiance * BRDF_Lambert( material.diffuseColor );
	reflectedLight.directSpecular += irradiance * BRDF_BlinnPhong( directLight.direction, geometryViewDir, geometryNormal, material.specularColor, material.specularShininess ) * material.specularStrength;
}
void RE_IndirectDiffuse_BlinnPhong( const in vec3 irradiance, const in vec3 geometryPosition, const in vec3 geometryNormal, const in vec3 geometryViewDir, const in vec3 geometryClearcoatNormal, const in BlinnPhongMaterial material, inout ReflectedLight reflectedLight ) {
	reflectedLight.indirectDiffuse += irradiance * BRDF_Lambert( material.diffuseColor );
}
#define RE_Direct				RE_Direct_BlinnPhong
#define RE_IndirectDiffuse		RE_IndirectDiffuse_BlinnPhong`,W1=`PhysicalMaterial material;
material.diffuseColor = diffuseColor.rgb * ( 1.0 - metalnessFactor );
vec3 dxy = max( abs( dFdx( nonPerturbedNormal ) ), abs( dFdy( nonPerturbedNormal ) ) );
float geometryRoughness = max( max( dxy.x, dxy.y ), dxy.z );
material.roughness = max( roughnessFactor, 0.0525 );material.roughness += geometryRoughness;
material.roughness = min( material.roughness, 1.0 );
#ifdef IOR
	material.ior = ior;
	#ifdef USE_SPECULAR
		float specularIntensityFactor = specularIntensity;
		vec3 specularColorFactor = specularColor;
		#ifdef USE_SPECULAR_COLORMAP
			specularColorFactor *= texture2D( specularColorMap, vSpecularColorMapUv ).rgb;
		#endif
		#ifdef USE_SPECULAR_INTENSITYMAP
			specularIntensityFactor *= texture2D( specularIntensityMap, vSpecularIntensityMapUv ).a;
		#endif
		material.specularF90 = mix( specularIntensityFactor, 1.0, metalnessFactor );
	#else
		float specularIntensityFactor = 1.0;
		vec3 specularColorFactor = vec3( 1.0 );
		material.specularF90 = 1.0;
	#endif
	material.specularColor = mix( min( pow2( ( material.ior - 1.0 ) / ( material.ior + 1.0 ) ) * specularColorFactor, vec3( 1.0 ) ) * specularIntensityFactor, diffuseColor.rgb, metalnessFactor );
#else
	material.specularColor = mix( vec3( 0.04 ), diffuseColor.rgb, metalnessFactor );
	material.specularF90 = 1.0;
#endif
#ifdef USE_CLEARCOAT
	material.clearcoat = clearcoat;
	material.clearcoatRoughness = clearcoatRoughness;
	material.clearcoatF0 = vec3( 0.04 );
	material.clearcoatF90 = 1.0;
	#ifdef USE_CLEARCOATMAP
		material.clearcoat *= texture2D( clearcoatMap, vClearcoatMapUv ).x;
	#endif
	#ifdef USE_CLEARCOAT_ROUGHNESSMAP
		material.clearcoatRoughness *= texture2D( clearcoatRoughnessMap, vClearcoatRoughnessMapUv ).y;
	#endif
	material.clearcoat = saturate( material.clearcoat );	material.clearcoatRoughness = max( material.clearcoatRoughness, 0.0525 );
	material.clearcoatRoughness += geometryRoughness;
	material.clearcoatRoughness = min( material.clearcoatRoughness, 1.0 );
#endif
#ifdef USE_DISPERSION
	material.dispersion = dispersion;
#endif
#ifdef USE_IRIDESCENCE
	material.iridescence = iridescence;
	material.iridescenceIOR = iridescenceIOR;
	#ifdef USE_IRIDESCENCEMAP
		material.iridescence *= texture2D( iridescenceMap, vIridescenceMapUv ).r;
	#endif
	#ifdef USE_IRIDESCENCE_THICKNESSMAP
		material.iridescenceThickness = (iridescenceThicknessMaximum - iridescenceThicknessMinimum) * texture2D( iridescenceThicknessMap, vIridescenceThicknessMapUv ).g + iridescenceThicknessMinimum;
	#else
		material.iridescenceThickness = iridescenceThicknessMaximum;
	#endif
#endif
#ifdef USE_SHEEN
	material.sheenColor = sheenColor;
	#ifdef USE_SHEEN_COLORMAP
		material.sheenColor *= texture2D( sheenColorMap, vSheenColorMapUv ).rgb;
	#endif
	material.sheenRoughness = clamp( sheenRoughness, 0.07, 1.0 );
	#ifdef USE_SHEEN_ROUGHNESSMAP
		material.sheenRoughness *= texture2D( sheenRoughnessMap, vSheenRoughnessMapUv ).a;
	#endif
#endif
#ifdef USE_ANISOTROPY
	#ifdef USE_ANISOTROPYMAP
		mat2 anisotropyMat = mat2( anisotropyVector.x, anisotropyVector.y, - anisotropyVector.y, anisotropyVector.x );
		vec3 anisotropyPolar = texture2D( anisotropyMap, vAnisotropyMapUv ).rgb;
		vec2 anisotropyV = anisotropyMat * normalize( 2.0 * anisotropyPolar.rg - vec2( 1.0 ) ) * anisotropyPolar.b;
	#else
		vec2 anisotropyV = anisotropyVector;
	#endif
	material.anisotropy = length( anisotropyV );
	if( material.anisotropy == 0.0 ) {
		anisotropyV = vec2( 1.0, 0.0 );
	} else {
		anisotropyV /= material.anisotropy;
		material.anisotropy = saturate( material.anisotropy );
	}
	material.alphaT = mix( pow2( material.roughness ), 1.0, pow2( material.anisotropy ) );
	material.anisotropyT = tbn[ 0 ] * anisotropyV.x + tbn[ 1 ] * anisotropyV.y;
	material.anisotropyB = tbn[ 1 ] * anisotropyV.x - tbn[ 0 ] * anisotropyV.y;
#endif`,X1=`struct PhysicalMaterial {
	vec3 diffuseColor;
	float roughness;
	vec3 specularColor;
	float specularF90;
	float dispersion;
	#ifdef USE_CLEARCOAT
		float clearcoat;
		float clearcoatRoughness;
		vec3 clearcoatF0;
		float clearcoatF90;
	#endif
	#ifdef USE_IRIDESCENCE
		float iridescence;
		float iridescenceIOR;
		float iridescenceThickness;
		vec3 iridescenceFresnel;
		vec3 iridescenceF0;
	#endif
	#ifdef USE_SHEEN
		vec3 sheenColor;
		float sheenRoughness;
	#endif
	#ifdef IOR
		float ior;
	#endif
	#ifdef USE_TRANSMISSION
		float transmission;
		float transmissionAlpha;
		float thickness;
		float attenuationDistance;
		vec3 attenuationColor;
	#endif
	#ifdef USE_ANISOTROPY
		float anisotropy;
		float alphaT;
		vec3 anisotropyT;
		vec3 anisotropyB;
	#endif
};
vec3 clearcoatSpecularDirect = vec3( 0.0 );
vec3 clearcoatSpecularIndirect = vec3( 0.0 );
vec3 sheenSpecularDirect = vec3( 0.0 );
vec3 sheenSpecularIndirect = vec3(0.0 );
vec3 Schlick_to_F0( const in vec3 f, const in float f90, const in float dotVH ) {
    float x = clamp( 1.0 - dotVH, 0.0, 1.0 );
    float x2 = x * x;
    float x5 = clamp( x * x2 * x2, 0.0, 0.9999 );
    return ( f - vec3( f90 ) * x5 ) / ( 1.0 - x5 );
}
float V_GGX_SmithCorrelated( const in float alpha, const in float dotNL, const in float dotNV ) {
	float a2 = pow2( alpha );
	float gv = dotNL * sqrt( a2 + ( 1.0 - a2 ) * pow2( dotNV ) );
	float gl = dotNV * sqrt( a2 + ( 1.0 - a2 ) * pow2( dotNL ) );
	return 0.5 / max( gv + gl, EPSILON );
}
float D_GGX( const in float alpha, const in float dotNH ) {
	float a2 = pow2( alpha );
	float denom = pow2( dotNH ) * ( a2 - 1.0 ) + 1.0;
	return RECIPROCAL_PI * a2 / pow2( denom );
}
#ifdef USE_ANISOTROPY
	float V_GGX_SmithCorrelated_Anisotropic( const in float alphaT, const in float alphaB, const in float dotTV, const in float dotBV, const in float dotTL, const in float dotBL, const in float dotNV, const in float dotNL ) {
		float gv = dotNL * length( vec3( alphaT * dotTV, alphaB * dotBV, dotNV ) );
		float gl = dotNV * length( vec3( alphaT * dotTL, alphaB * dotBL, dotNL ) );
		float v = 0.5 / ( gv + gl );
		return saturate(v);
	}
	float D_GGX_Anisotropic( const in float alphaT, const in float alphaB, const in float dotNH, const in float dotTH, const in float dotBH ) {
		float a2 = alphaT * alphaB;
		highp vec3 v = vec3( alphaB * dotTH, alphaT * dotBH, a2 * dotNH );
		highp float v2 = dot( v, v );
		float w2 = a2 / v2;
		return RECIPROCAL_PI * a2 * pow2 ( w2 );
	}
#endif
#ifdef USE_CLEARCOAT
	vec3 BRDF_GGX_Clearcoat( const in vec3 lightDir, const in vec3 viewDir, const in vec3 normal, const in PhysicalMaterial material) {
		vec3 f0 = material.clearcoatF0;
		float f90 = material.clearcoatF90;
		float roughness = material.clearcoatRoughness;
		float alpha = pow2( roughness );
		vec3 halfDir = normalize( lightDir + viewDir );
		float dotNL = saturate( dot( normal, lightDir ) );
		float dotNV = saturate( dot( normal, viewDir ) );
		float dotNH = saturate( dot( normal, halfDir ) );
		float dotVH = saturate( dot( viewDir, halfDir ) );
		vec3 F = F_Schlick( f0, f90, dotVH );
		float V = V_GGX_SmithCorrelated( alpha, dotNL, dotNV );
		float D = D_GGX( alpha, dotNH );
		return F * ( V * D );
	}
#endif
vec3 BRDF_GGX( const in vec3 lightDir, const in vec3 viewDir, const in vec3 normal, const in PhysicalMaterial material ) {
	vec3 f0 = material.specularColor;
	float f90 = material.specularF90;
	float roughness = material.roughness;
	float alpha = pow2( roughness );
	vec3 halfDir = normalize( lightDir + viewDir );
	float dotNL = saturate( dot( normal, lightDir ) );
	float dotNV = saturate( dot( normal, viewDir ) );
	float dotNH = saturate( dot( normal, halfDir ) );
	float dotVH = saturate( dot( viewDir, halfDir ) );
	vec3 F = F_Schlick( f0, f90, dotVH );
	#ifdef USE_IRIDESCENCE
		F = mix( F, material.iridescenceFresnel, material.iridescence );
	#endif
	#ifdef USE_ANISOTROPY
		float dotTL = dot( material.anisotropyT, lightDir );
		float dotTV = dot( material.anisotropyT, viewDir );
		float dotTH = dot( material.anisotropyT, halfDir );
		float dotBL = dot( material.anisotropyB, lightDir );
		float dotBV = dot( material.anisotropyB, viewDir );
		float dotBH = dot( material.anisotropyB, halfDir );
		float V = V_GGX_SmithCorrelated_Anisotropic( material.alphaT, alpha, dotTV, dotBV, dotTL, dotBL, dotNV, dotNL );
		float D = D_GGX_Anisotropic( material.alphaT, alpha, dotNH, dotTH, dotBH );
	#else
		float V = V_GGX_SmithCorrelated( alpha, dotNL, dotNV );
		float D = D_GGX( alpha, dotNH );
	#endif
	return F * ( V * D );
}
vec2 LTC_Uv( const in vec3 N, const in vec3 V, const in float roughness ) {
	const float LUT_SIZE = 64.0;
	const float LUT_SCALE = ( LUT_SIZE - 1.0 ) / LUT_SIZE;
	const float LUT_BIAS = 0.5 / LUT_SIZE;
	float dotNV = saturate( dot( N, V ) );
	vec2 uv = vec2( roughness, sqrt( 1.0 - dotNV ) );
	uv = uv * LUT_SCALE + LUT_BIAS;
	return uv;
}
float LTC_ClippedSphereFormFactor( const in vec3 f ) {
	float l = length( f );
	return max( ( l * l + f.z ) / ( l + 1.0 ), 0.0 );
}
vec3 LTC_EdgeVectorFormFactor( const in vec3 v1, const in vec3 v2 ) {
	float x = dot( v1, v2 );
	float y = abs( x );
	float a = 0.8543985 + ( 0.4965155 + 0.0145206 * y ) * y;
	float b = 3.4175940 + ( 4.1616724 + y ) * y;
	float v = a / b;
	float theta_sintheta = ( x > 0.0 ) ? v : 0.5 * inversesqrt( max( 1.0 - x * x, 1e-7 ) ) - v;
	return cross( v1, v2 ) * theta_sintheta;
}
vec3 LTC_Evaluate( const in vec3 N, const in vec3 V, const in vec3 P, const in mat3 mInv, const in vec3 rectCoords[ 4 ] ) {
	vec3 v1 = rectCoords[ 1 ] - rectCoords[ 0 ];
	vec3 v2 = rectCoords[ 3 ] - rectCoords[ 0 ];
	vec3 lightNormal = cross( v1, v2 );
	if( dot( lightNormal, P - rectCoords[ 0 ] ) < 0.0 ) return vec3( 0.0 );
	vec3 T1, T2;
	T1 = normalize( V - N * dot( V, N ) );
	T2 = - cross( N, T1 );
	mat3 mat = mInv * transposeMat3( mat3( T1, T2, N ) );
	vec3 coords[ 4 ];
	coords[ 0 ] = mat * ( rectCoords[ 0 ] - P );
	coords[ 1 ] = mat * ( rectCoords[ 1 ] - P );
	coords[ 2 ] = mat * ( rectCoords[ 2 ] - P );
	coords[ 3 ] = mat * ( rectCoords[ 3 ] - P );
	coords[ 0 ] = normalize( coords[ 0 ] );
	coords[ 1 ] = normalize( coords[ 1 ] );
	coords[ 2 ] = normalize( coords[ 2 ] );
	coords[ 3 ] = normalize( coords[ 3 ] );
	vec3 vectorFormFactor = vec3( 0.0 );
	vectorFormFactor += LTC_EdgeVectorFormFactor( coords[ 0 ], coords[ 1 ] );
	vectorFormFactor += LTC_EdgeVectorFormFactor( coords[ 1 ], coords[ 2 ] );
	vectorFormFactor += LTC_EdgeVectorFormFactor( coords[ 2 ], coords[ 3 ] );
	vectorFormFactor += LTC_EdgeVectorFormFactor( coords[ 3 ], coords[ 0 ] );
	float result = LTC_ClippedSphereFormFactor( vectorFormFactor );
	return vec3( result );
}
#if defined( USE_SHEEN )
float D_Charlie( float roughness, float dotNH ) {
	float alpha = pow2( roughness );
	float invAlpha = 1.0 / alpha;
	float cos2h = dotNH * dotNH;
	float sin2h = max( 1.0 - cos2h, 0.0078125 );
	return ( 2.0 + invAlpha ) * pow( sin2h, invAlpha * 0.5 ) / ( 2.0 * PI );
}
float V_Neubelt( float dotNV, float dotNL ) {
	return saturate( 1.0 / ( 4.0 * ( dotNL + dotNV - dotNL * dotNV ) ) );
}
vec3 BRDF_Sheen( const in vec3 lightDir, const in vec3 viewDir, const in vec3 normal, vec3 sheenColor, const in float sheenRoughness ) {
	vec3 halfDir = normalize( lightDir + viewDir );
	float dotNL = saturate( dot( normal, lightDir ) );
	float dotNV = saturate( dot( normal, viewDir ) );
	float dotNH = saturate( dot( normal, halfDir ) );
	float D = D_Charlie( sheenRoughness, dotNH );
	float V = V_Neubelt( dotNV, dotNL );
	return sheenColor * ( D * V );
}
#endif
float IBLSheenBRDF( const in vec3 normal, const in vec3 viewDir, const in float roughness ) {
	float dotNV = saturate( dot( normal, viewDir ) );
	float r2 = roughness * roughness;
	float a = roughness < 0.25 ? -339.2 * r2 + 161.4 * roughness - 25.9 : -8.48 * r2 + 14.3 * roughness - 9.95;
	float b = roughness < 0.25 ? 44.0 * r2 - 23.7 * roughness + 3.26 : 1.97 * r2 - 3.27 * roughness + 0.72;
	float DG = exp( a * dotNV + b ) + ( roughness < 0.25 ? 0.0 : 0.1 * ( roughness - 0.25 ) );
	return saturate( DG * RECIPROCAL_PI );
}
vec2 DFGApprox( const in vec3 normal, const in vec3 viewDir, const in float roughness ) {
	float dotNV = saturate( dot( normal, viewDir ) );
	const vec4 c0 = vec4( - 1, - 0.0275, - 0.572, 0.022 );
	const vec4 c1 = vec4( 1, 0.0425, 1.04, - 0.04 );
	vec4 r = roughness * c0 + c1;
	float a004 = min( r.x * r.x, exp2( - 9.28 * dotNV ) ) * r.x + r.y;
	vec2 fab = vec2( - 1.04, 1.04 ) * a004 + r.zw;
	return fab;
}
vec3 EnvironmentBRDF( const in vec3 normal, const in vec3 viewDir, const in vec3 specularColor, const in float specularF90, const in float roughness ) {
	vec2 fab = DFGApprox( normal, viewDir, roughness );
	return specularColor * fab.x + specularF90 * fab.y;
}
#ifdef USE_IRIDESCENCE
void computeMultiscatteringIridescence( const in vec3 normal, const in vec3 viewDir, const in vec3 specularColor, const in float specularF90, const in float iridescence, const in vec3 iridescenceF0, const in float roughness, inout vec3 singleScatter, inout vec3 multiScatter ) {
#else
void computeMultiscattering( const in vec3 normal, const in vec3 viewDir, const in vec3 specularColor, const in float specularF90, const in float roughness, inout vec3 singleScatter, inout vec3 multiScatter ) {
#endif
	vec2 fab = DFGApprox( normal, viewDir, roughness );
	#ifdef USE_IRIDESCENCE
		vec3 Fr = mix( specularColor, iridescenceF0, iridescence );
	#else
		vec3 Fr = specularColor;
	#endif
	vec3 FssEss = Fr * fab.x + specularF90 * fab.y;
	float Ess = fab.x + fab.y;
	float Ems = 1.0 - Ess;
	vec3 Favg = Fr + ( 1.0 - Fr ) * 0.047619;	vec3 Fms = FssEss * Favg / ( 1.0 - Ems * Favg );
	singleScatter += FssEss;
	multiScatter += Fms * Ems;
}
#if NUM_RECT_AREA_LIGHTS > 0
	void RE_Direct_RectArea_Physical( const in RectAreaLight rectAreaLight, const in vec3 geometryPosition, const in vec3 geometryNormal, const in vec3 geometryViewDir, const in vec3 geometryClearcoatNormal, const in PhysicalMaterial material, inout ReflectedLight reflectedLight ) {
		vec3 normal = geometryNormal;
		vec3 viewDir = geometryViewDir;
		vec3 position = geometryPosition;
		vec3 lightPos = rectAreaLight.position;
		vec3 halfWidth = rectAreaLight.halfWidth;
		vec3 halfHeight = rectAreaLight.halfHeight;
		vec3 lightColor = rectAreaLight.color;
		float roughness = material.roughness;
		vec3 rectCoords[ 4 ];
		rectCoords[ 0 ] = lightPos + halfWidth - halfHeight;		rectCoords[ 1 ] = lightPos - halfWidth - halfHeight;
		rectCoords[ 2 ] = lightPos - halfWidth + halfHeight;
		rectCoords[ 3 ] = lightPos + halfWidth + halfHeight;
		vec2 uv = LTC_Uv( normal, viewDir, roughness );
		vec4 t1 = texture2D( ltc_1, uv );
		vec4 t2 = texture2D( ltc_2, uv );
		mat3 mInv = mat3(
			vec3( t1.x, 0, t1.y ),
			vec3(    0, 1,    0 ),
			vec3( t1.z, 0, t1.w )
		);
		vec3 fresnel = ( material.specularColor * t2.x + ( vec3( 1.0 ) - material.specularColor ) * t2.y );
		reflectedLight.directSpecular += lightColor * fresnel * LTC_Evaluate( normal, viewDir, position, mInv, rectCoords );
		reflectedLight.directDiffuse += lightColor * material.diffuseColor * LTC_Evaluate( normal, viewDir, position, mat3( 1.0 ), rectCoords );
	}
#endif
void RE_Direct_Physical( const in IncidentLight directLight, const in vec3 geometryPosition, const in vec3 geometryNormal, const in vec3 geometryViewDir, const in vec3 geometryClearcoatNormal, const in PhysicalMaterial material, inout ReflectedLight reflectedLight ) {
	float dotNL = saturate( dot( geometryNormal, directLight.direction ) );
	vec3 irradiance = dotNL * directLight.color;
	#ifdef USE_CLEARCOAT
		float dotNLcc = saturate( dot( geometryClearcoatNormal, directLight.direction ) );
		vec3 ccIrradiance = dotNLcc * directLight.color;
		clearcoatSpecularDirect += ccIrradiance * BRDF_GGX_Clearcoat( directLight.direction, geometryViewDir, geometryClearcoatNormal, material );
	#endif
	#ifdef USE_SHEEN
		sheenSpecularDirect += irradiance * BRDF_Sheen( directLight.direction, geometryViewDir, geometryNormal, material.sheenColor, material.sheenRoughness );
	#endif
	reflectedLight.directSpecular += irradiance * BRDF_GGX( directLight.direction, geometryViewDir, geometryNormal, material );
	reflectedLight.directDiffuse += irradiance * BRDF_Lambert( material.diffuseColor );
}
void RE_IndirectDiffuse_Physical( const in vec3 irradiance, const in vec3 geometryPosition, const in vec3 geometryNormal, const in vec3 geometryViewDir, const in vec3 geometryClearcoatNormal, const in PhysicalMaterial material, inout ReflectedLight reflectedLight ) {
	reflectedLight.indirectDiffuse += irradiance * BRDF_Lambert( material.diffuseColor );
}
void RE_IndirectSpecular_Physical( const in vec3 radiance, const in vec3 irradiance, const in vec3 clearcoatRadiance, const in vec3 geometryPosition, const in vec3 geometryNormal, const in vec3 geometryViewDir, const in vec3 geometryClearcoatNormal, const in PhysicalMaterial material, inout ReflectedLight reflectedLight) {
	#ifdef USE_CLEARCOAT
		clearcoatSpecularIndirect += clearcoatRadiance * EnvironmentBRDF( geometryClearcoatNormal, geometryViewDir, material.clearcoatF0, material.clearcoatF90, material.clearcoatRoughness );
	#endif
	#ifdef USE_SHEEN
		sheenSpecularIndirect += irradiance * material.sheenColor * IBLSheenBRDF( geometryNormal, geometryViewDir, material.sheenRoughness );
	#endif
	vec3 singleScattering = vec3( 0.0 );
	vec3 multiScattering = vec3( 0.0 );
	vec3 cosineWeightedIrradiance = irradiance * RECIPROCAL_PI;
	#ifdef USE_IRIDESCENCE
		computeMultiscatteringIridescence( geometryNormal, geometryViewDir, material.specularColor, material.specularF90, material.iridescence, material.iridescenceFresnel, material.roughness, singleScattering, multiScattering );
	#else
		computeMultiscattering( geometryNormal, geometryViewDir, material.specularColor, material.specularF90, material.roughness, singleScattering, multiScattering );
	#endif
	vec3 totalScattering = singleScattering + multiScattering;
	vec3 diffuse = material.diffuseColor * ( 1.0 - max( max( totalScattering.r, totalScattering.g ), totalScattering.b ) );
	reflectedLight.indirectSpecular += radiance * singleScattering;
	reflectedLight.indirectSpecular += multiScattering * cosineWeightedIrradiance;
	reflectedLight.indirectDiffuse += diffuse * cosineWeightedIrradiance;
}
#define RE_Direct				RE_Direct_Physical
#define RE_Direct_RectArea		RE_Direct_RectArea_Physical
#define RE_IndirectDiffuse		RE_IndirectDiffuse_Physical
#define RE_IndirectSpecular		RE_IndirectSpecular_Physical
float computeSpecularOcclusion( const in float dotNV, const in float ambientOcclusion, const in float roughness ) {
	return saturate( pow( dotNV + ambientOcclusion, exp2( - 16.0 * roughness - 1.0 ) ) - 1.0 + ambientOcclusion );
}`,Y1=`
vec3 geometryPosition = - vViewPosition;
vec3 geometryNormal = normal;
vec3 geometryViewDir = ( isOrthographic ) ? vec3( 0, 0, 1 ) : normalize( vViewPosition );
vec3 geometryClearcoatNormal = vec3( 0.0 );
#ifdef USE_CLEARCOAT
	geometryClearcoatNormal = clearcoatNormal;
#endif
#ifdef USE_IRIDESCENCE
	float dotNVi = saturate( dot( normal, geometryViewDir ) );
	if ( material.iridescenceThickness == 0.0 ) {
		material.iridescence = 0.0;
	} else {
		material.iridescence = saturate( material.iridescence );
	}
	if ( material.iridescence > 0.0 ) {
		material.iridescenceFresnel = evalIridescence( 1.0, material.iridescenceIOR, dotNVi, material.iridescenceThickness, material.specularColor );
		material.iridescenceF0 = Schlick_to_F0( material.iridescenceFresnel, 1.0, dotNVi );
	}
#endif
IncidentLight directLight;
#if ( NUM_POINT_LIGHTS > 0 ) && defined( RE_Direct )
	PointLight pointLight;
	#if defined( USE_SHADOWMAP ) && NUM_POINT_LIGHT_SHADOWS > 0
	PointLightShadow pointLightShadow;
	#endif
	#pragma unroll_loop_start
	for ( int i = 0; i < NUM_POINT_LIGHTS; i ++ ) {
		pointLight = pointLights[ i ];
		getPointLightInfo( pointLight, geometryPosition, directLight );
		#if defined( USE_SHADOWMAP ) && ( UNROLLED_LOOP_INDEX < NUM_POINT_LIGHT_SHADOWS )
		pointLightShadow = pointLightShadows[ i ];
		directLight.color *= ( directLight.visible && receiveShadow ) ? getPointShadow( pointShadowMap[ i ], pointLightShadow.shadowMapSize, pointLightShadow.shadowIntensity, pointLightShadow.shadowBias, pointLightShadow.shadowRadius, vPointShadowCoord[ i ], pointLightShadow.shadowCameraNear, pointLightShadow.shadowCameraFar ) : 1.0;
		#endif
		RE_Direct( directLight, geometryPosition, geometryNormal, geometryViewDir, geometryClearcoatNormal, material, reflectedLight );
	}
	#pragma unroll_loop_end
#endif
#if ( NUM_SPOT_LIGHTS > 0 ) && defined( RE_Direct )
	SpotLight spotLight;
	vec4 spotColor;
	vec3 spotLightCoord;
	bool inSpotLightMap;
	#if defined( USE_SHADOWMAP ) && NUM_SPOT_LIGHT_SHADOWS > 0
	SpotLightShadow spotLightShadow;
	#endif
	#pragma unroll_loop_start
	for ( int i = 0; i < NUM_SPOT_LIGHTS; i ++ ) {
		spotLight = spotLights[ i ];
		getSpotLightInfo( spotLight, geometryPosition, directLight );
		#if ( UNROLLED_LOOP_INDEX < NUM_SPOT_LIGHT_SHADOWS_WITH_MAPS )
		#define SPOT_LIGHT_MAP_INDEX UNROLLED_LOOP_INDEX
		#elif ( UNROLLED_LOOP_INDEX < NUM_SPOT_LIGHT_SHADOWS )
		#define SPOT_LIGHT_MAP_INDEX NUM_SPOT_LIGHT_MAPS
		#else
		#define SPOT_LIGHT_MAP_INDEX ( UNROLLED_LOOP_INDEX - NUM_SPOT_LIGHT_SHADOWS + NUM_SPOT_LIGHT_SHADOWS_WITH_MAPS )
		#endif
		#if ( SPOT_LIGHT_MAP_INDEX < NUM_SPOT_LIGHT_MAPS )
			spotLightCoord = vSpotLightCoord[ i ].xyz / vSpotLightCoord[ i ].w;
			inSpotLightMap = all( lessThan( abs( spotLightCoord * 2. - 1. ), vec3( 1.0 ) ) );
			spotColor = texture2D( spotLightMap[ SPOT_LIGHT_MAP_INDEX ], spotLightCoord.xy );
			directLight.color = inSpotLightMap ? directLight.color * spotColor.rgb : directLight.color;
		#endif
		#undef SPOT_LIGHT_MAP_INDEX
		#if defined( USE_SHADOWMAP ) && ( UNROLLED_LOOP_INDEX < NUM_SPOT_LIGHT_SHADOWS )
		spotLightShadow = spotLightShadows[ i ];
		directLight.color *= ( directLight.visible && receiveShadow ) ? getShadow( spotShadowMap[ i ], spotLightShadow.shadowMapSize, spotLightShadow.shadowIntensity, spotLightShadow.shadowBias, spotLightShadow.shadowRadius, vSpotLightCoord[ i ] ) : 1.0;
		#endif
		RE_Direct( directLight, geometryPosition, geometryNormal, geometryViewDir, geometryClearcoatNormal, material, reflectedLight );
	}
	#pragma unroll_loop_end
#endif
#if ( NUM_DIR_LIGHTS > 0 ) && defined( RE_Direct )
	DirectionalLight directionalLight;
	#if defined( USE_SHADOWMAP ) && NUM_DIR_LIGHT_SHADOWS > 0
	DirectionalLightShadow directionalLightShadow;
	#endif
	#pragma unroll_loop_start
	for ( int i = 0; i < NUM_DIR_LIGHTS; i ++ ) {
		directionalLight = directionalLights[ i ];
		getDirectionalLightInfo( directionalLight, directLight );
		#if defined( USE_SHADOWMAP ) && ( UNROLLED_LOOP_INDEX < NUM_DIR_LIGHT_SHADOWS )
		directionalLightShadow = directionalLightShadows[ i ];
		directLight.color *= ( directLight.visible && receiveShadow ) ? getShadow( directionalShadowMap[ i ], directionalLightShadow.shadowMapSize, directionalLightShadow.shadowIntensity, directionalLightShadow.shadowBias, directionalLightShadow.shadowRadius, vDirectionalShadowCoord[ i ] ) : 1.0;
		#endif
		RE_Direct( directLight, geometryPosition, geometryNormal, geometryViewDir, geometryClearcoatNormal, material, reflectedLight );
	}
	#pragma unroll_loop_end
#endif
#if ( NUM_RECT_AREA_LIGHTS > 0 ) && defined( RE_Direct_RectArea )
	RectAreaLight rectAreaLight;
	#pragma unroll_loop_start
	for ( int i = 0; i < NUM_RECT_AREA_LIGHTS; i ++ ) {
		rectAreaLight = rectAreaLights[ i ];
		RE_Direct_RectArea( rectAreaLight, geometryPosition, geometryNormal, geometryViewDir, geometryClearcoatNormal, material, reflectedLight );
	}
	#pragma unroll_loop_end
#endif
#if defined( RE_IndirectDiffuse )
	vec3 iblIrradiance = vec3( 0.0 );
	vec3 irradiance = getAmbientLightIrradiance( ambientLightColor );
	#if defined( USE_LIGHT_PROBES )
		irradiance += getLightProbeIrradiance( lightProbe, geometryNormal );
	#endif
	#if ( NUM_HEMI_LIGHTS > 0 )
		#pragma unroll_loop_start
		for ( int i = 0; i < NUM_HEMI_LIGHTS; i ++ ) {
			irradiance += getHemisphereLightIrradiance( hemisphereLights[ i ], geometryNormal );
		}
		#pragma unroll_loop_end
	#endif
#endif
#if defined( RE_IndirectSpecular )
	vec3 radiance = vec3( 0.0 );
	vec3 clearcoatRadiance = vec3( 0.0 );
#endif`,q1=`#if defined( RE_IndirectDiffuse )
	#ifdef USE_LIGHTMAP
		vec4 lightMapTexel = texture2D( lightMap, vLightMapUv );
		vec3 lightMapIrradiance = lightMapTexel.rgb * lightMapIntensity;
		irradiance += lightMapIrradiance;
	#endif
	#if defined( USE_ENVMAP ) && defined( STANDARD ) && defined( ENVMAP_TYPE_CUBE_UV )
		iblIrradiance += getIBLIrradiance( geometryNormal );
	#endif
#endif
#if defined( USE_ENVMAP ) && defined( RE_IndirectSpecular )
	#ifdef USE_ANISOTROPY
		radiance += getIBLAnisotropyRadiance( geometryViewDir, geometryNormal, material.roughness, material.anisotropyB, material.anisotropy );
	#else
		radiance += getIBLRadiance( geometryViewDir, geometryNormal, material.roughness );
	#endif
	#ifdef USE_CLEARCOAT
		clearcoatRadiance += getIBLRadiance( geometryViewDir, geometryClearcoatNormal, material.clearcoatRoughness );
	#endif
#endif`,$1=`#if defined( RE_IndirectDiffuse )
	RE_IndirectDiffuse( irradiance, geometryPosition, geometryNormal, geometryViewDir, geometryClearcoatNormal, material, reflectedLight );
#endif
#if defined( RE_IndirectSpecular )
	RE_IndirectSpecular( radiance, iblIrradiance, clearcoatRadiance, geometryPosition, geometryNormal, geometryViewDir, geometryClearcoatNormal, material, reflectedLight );
#endif`,Z1=`#if defined( USE_LOGDEPTHBUF )
	gl_FragDepth = vIsPerspective == 0.0 ? gl_FragCoord.z : log2( vFragDepth ) * logDepthBufFC * 0.5;
#endif`,J1=`#if defined( USE_LOGDEPTHBUF )
	uniform float logDepthBufFC;
	varying float vFragDepth;
	varying float vIsPerspective;
#endif`,K1=`#ifdef USE_LOGDEPTHBUF
	varying float vFragDepth;
	varying float vIsPerspective;
#endif`,j1=`#ifdef USE_LOGDEPTHBUF
	vFragDepth = 1.0 + gl_Position.w;
	vIsPerspective = float( isPerspectiveMatrix( projectionMatrix ) );
#endif`,Q1=`#ifdef USE_MAP
	vec4 sampledDiffuseColor = texture2D( map, vMapUv );
	#ifdef DECODE_VIDEO_TEXTURE
		sampledDiffuseColor = vec4( mix( pow( sampledDiffuseColor.rgb * 0.9478672986 + vec3( 0.0521327014 ), vec3( 2.4 ) ), sampledDiffuseColor.rgb * 0.0773993808, vec3( lessThanEqual( sampledDiffuseColor.rgb, vec3( 0.04045 ) ) ) ), sampledDiffuseColor.w );
	
	#endif
	diffuseColor *= sampledDiffuseColor;
#endif`,eM=`#ifdef USE_MAP
	uniform sampler2D map;
#endif`,tM=`#if defined( USE_MAP ) || defined( USE_ALPHAMAP )
	#if defined( USE_POINTS_UV )
		vec2 uv = vUv;
	#else
		vec2 uv = ( uvTransform * vec3( gl_PointCoord.x, 1.0 - gl_PointCoord.y, 1 ) ).xy;
	#endif
#endif
#ifdef USE_MAP
	diffuseColor *= texture2D( map, uv );
#endif
#ifdef USE_ALPHAMAP
	diffuseColor.a *= texture2D( alphaMap, uv ).g;
#endif`,nM=`#if defined( USE_POINTS_UV )
	varying vec2 vUv;
#else
	#if defined( USE_MAP ) || defined( USE_ALPHAMAP )
		uniform mat3 uvTransform;
	#endif
#endif
#ifdef USE_MAP
	uniform sampler2D map;
#endif
#ifdef USE_ALPHAMAP
	uniform sampler2D alphaMap;
#endif`,iM=`float metalnessFactor = metalness;
#ifdef USE_METALNESSMAP
	vec4 texelMetalness = texture2D( metalnessMap, vMetalnessMapUv );
	metalnessFactor *= texelMetalness.b;
#endif`,rM=`#ifdef USE_METALNESSMAP
	uniform sampler2D metalnessMap;
#endif`,sM=`#ifdef USE_INSTANCING_MORPH
	float morphTargetInfluences[ MORPHTARGETS_COUNT ];
	float morphTargetBaseInfluence = texelFetch( morphTexture, ivec2( 0, gl_InstanceID ), 0 ).r;
	for ( int i = 0; i < MORPHTARGETS_COUNT; i ++ ) {
		morphTargetInfluences[i] =  texelFetch( morphTexture, ivec2( i + 1, gl_InstanceID ), 0 ).r;
	}
#endif`,oM=`#if defined( USE_MORPHCOLORS )
	vColor *= morphTargetBaseInfluence;
	for ( int i = 0; i < MORPHTARGETS_COUNT; i ++ ) {
		#if defined( USE_COLOR_ALPHA )
			if ( morphTargetInfluences[ i ] != 0.0 ) vColor += getMorph( gl_VertexID, i, 2 ) * morphTargetInfluences[ i ];
		#elif defined( USE_COLOR )
			if ( morphTargetInfluences[ i ] != 0.0 ) vColor += getMorph( gl_VertexID, i, 2 ).rgb * morphTargetInfluences[ i ];
		#endif
	}
#endif`,aM=`#ifdef USE_MORPHNORMALS
	objectNormal *= morphTargetBaseInfluence;
	for ( int i = 0; i < MORPHTARGETS_COUNT; i ++ ) {
		if ( morphTargetInfluences[ i ] != 0.0 ) objectNormal += getMorph( gl_VertexID, i, 1 ).xyz * morphTargetInfluences[ i ];
	}
#endif`,lM=`#ifdef USE_MORPHTARGETS
	#ifndef USE_INSTANCING_MORPH
		uniform float morphTargetBaseInfluence;
		uniform float morphTargetInfluences[ MORPHTARGETS_COUNT ];
	#endif
	uniform sampler2DArray morphTargetsTexture;
	uniform ivec2 morphTargetsTextureSize;
	vec4 getMorph( const in int vertexIndex, const in int morphTargetIndex, const in int offset ) {
		int texelIndex = vertexIndex * MORPHTARGETS_TEXTURE_STRIDE + offset;
		int y = texelIndex / morphTargetsTextureSize.x;
		int x = texelIndex - y * morphTargetsTextureSize.x;
		ivec3 morphUV = ivec3( x, y, morphTargetIndex );
		return texelFetch( morphTargetsTexture, morphUV, 0 );
	}
#endif`,cM=`#ifdef USE_MORPHTARGETS
	transformed *= morphTargetBaseInfluence;
	for ( int i = 0; i < MORPHTARGETS_COUNT; i ++ ) {
		if ( morphTargetInfluences[ i ] != 0.0 ) transformed += getMorph( gl_VertexID, i, 0 ).xyz * morphTargetInfluences[ i ];
	}
#endif`,uM=`float faceDirection = gl_FrontFacing ? 1.0 : - 1.0;
#ifdef FLAT_SHADED
	vec3 fdx = dFdx( vViewPosition );
	vec3 fdy = dFdy( vViewPosition );
	vec3 normal = normalize( cross( fdx, fdy ) );
#else
	vec3 normal = normalize( vNormal );
	#ifdef DOUBLE_SIDED
		normal *= faceDirection;
	#endif
#endif
#if defined( USE_NORMALMAP_TANGENTSPACE ) || defined( USE_CLEARCOAT_NORMALMAP ) || defined( USE_ANISOTROPY )
	#ifdef USE_TANGENT
		mat3 tbn = mat3( normalize( vTangent ), normalize( vBitangent ), normal );
	#else
		mat3 tbn = getTangentFrame( - vViewPosition, normal,
		#if defined( USE_NORMALMAP )
			vNormalMapUv
		#elif defined( USE_CLEARCOAT_NORMALMAP )
			vClearcoatNormalMapUv
		#else
			vUv
		#endif
		);
	#endif
	#if defined( DOUBLE_SIDED ) && ! defined( FLAT_SHADED )
		tbn[0] *= faceDirection;
		tbn[1] *= faceDirection;
	#endif
#endif
#ifdef USE_CLEARCOAT_NORMALMAP
	#ifdef USE_TANGENT
		mat3 tbn2 = mat3( normalize( vTangent ), normalize( vBitangent ), normal );
	#else
		mat3 tbn2 = getTangentFrame( - vViewPosition, normal, vClearcoatNormalMapUv );
	#endif
	#if defined( DOUBLE_SIDED ) && ! defined( FLAT_SHADED )
		tbn2[0] *= faceDirection;
		tbn2[1] *= faceDirection;
	#endif
#endif
vec3 nonPerturbedNormal = normal;`,hM=`#ifdef USE_NORMALMAP_OBJECTSPACE
	normal = texture2D( normalMap, vNormalMapUv ).xyz * 2.0 - 1.0;
	#ifdef FLIP_SIDED
		normal = - normal;
	#endif
	#ifdef DOUBLE_SIDED
		normal = normal * faceDirection;
	#endif
	normal = normalize( normalMatrix * normal );
#elif defined( USE_NORMALMAP_TANGENTSPACE )
	vec3 mapN = texture2D( normalMap, vNormalMapUv ).xyz * 2.0 - 1.0;
	mapN.xy *= normalScale;
	normal = normalize( tbn * mapN );
#elif defined( USE_BUMPMAP )
	normal = perturbNormalArb( - vViewPosition, normal, dHdxy_fwd(), faceDirection );
#endif`,dM=`#ifndef FLAT_SHADED
	varying vec3 vNormal;
	#ifdef USE_TANGENT
		varying vec3 vTangent;
		varying vec3 vBitangent;
	#endif
#endif`,fM=`#ifndef FLAT_SHADED
	varying vec3 vNormal;
	#ifdef USE_TANGENT
		varying vec3 vTangent;
		varying vec3 vBitangent;
	#endif
#endif`,pM=`#ifndef FLAT_SHADED
	vNormal = normalize( transformedNormal );
	#ifdef USE_TANGENT
		vTangent = normalize( transformedTangent );
		vBitangent = normalize( cross( vNormal, vTangent ) * tangent.w );
	#endif
#endif`,mM=`#ifdef USE_NORMALMAP
	uniform sampler2D normalMap;
	uniform vec2 normalScale;
#endif
#ifdef USE_NORMALMAP_OBJECTSPACE
	uniform mat3 normalMatrix;
#endif
#if ! defined ( USE_TANGENT ) && ( defined ( USE_NORMALMAP_TANGENTSPACE ) || defined ( USE_CLEARCOAT_NORMALMAP ) || defined( USE_ANISOTROPY ) )
	mat3 getTangentFrame( vec3 eye_pos, vec3 surf_norm, vec2 uv ) {
		vec3 q0 = dFdx( eye_pos.xyz );
		vec3 q1 = dFdy( eye_pos.xyz );
		vec2 st0 = dFdx( uv.st );
		vec2 st1 = dFdy( uv.st );
		vec3 N = surf_norm;
		vec3 q1perp = cross( q1, N );
		vec3 q0perp = cross( N, q0 );
		vec3 T = q1perp * st0.x + q0perp * st1.x;
		vec3 B = q1perp * st0.y + q0perp * st1.y;
		float det = max( dot( T, T ), dot( B, B ) );
		float scale = ( det == 0.0 ) ? 0.0 : inversesqrt( det );
		return mat3( T * scale, B * scale, N );
	}
#endif`,gM=`#ifdef USE_CLEARCOAT
	vec3 clearcoatNormal = nonPerturbedNormal;
#endif`,_M=`#ifdef USE_CLEARCOAT_NORMALMAP
	vec3 clearcoatMapN = texture2D( clearcoatNormalMap, vClearcoatNormalMapUv ).xyz * 2.0 - 1.0;
	clearcoatMapN.xy *= clearcoatNormalScale;
	clearcoatNormal = normalize( tbn2 * clearcoatMapN );
#endif`,vM=`#ifdef USE_CLEARCOATMAP
	uniform sampler2D clearcoatMap;
#endif
#ifdef USE_CLEARCOAT_NORMALMAP
	uniform sampler2D clearcoatNormalMap;
	uniform vec2 clearcoatNormalScale;
#endif
#ifdef USE_CLEARCOAT_ROUGHNESSMAP
	uniform sampler2D clearcoatRoughnessMap;
#endif`,yM=`#ifdef USE_IRIDESCENCEMAP
	uniform sampler2D iridescenceMap;
#endif
#ifdef USE_IRIDESCENCE_THICKNESSMAP
	uniform sampler2D iridescenceThicknessMap;
#endif`,xM=`#ifdef OPAQUE
diffuseColor.a = 1.0;
#endif
#ifdef USE_TRANSMISSION
diffuseColor.a *= material.transmissionAlpha;
#endif
gl_FragColor = vec4( outgoingLight, diffuseColor.a );`,wM=`vec3 packNormalToRGB( const in vec3 normal ) {
	return normalize( normal ) * 0.5 + 0.5;
}
vec3 unpackRGBToNormal( const in vec3 rgb ) {
	return 2.0 * rgb.xyz - 1.0;
}
const float PackUpscale = 256. / 255.;const float UnpackDownscale = 255. / 256.;
const vec3 PackFactors = vec3( 256. * 256. * 256., 256. * 256., 256. );
const vec4 UnpackFactors = UnpackDownscale / vec4( PackFactors, 1. );
const float ShiftRight8 = 1. / 256.;
vec4 packDepthToRGBA( const in float v ) {
	vec4 r = vec4( fract( v * PackFactors ), v );
	r.yzw -= r.xyz * ShiftRight8;	return r * PackUpscale;
}
float unpackRGBAToDepth( const in vec4 v ) {
	return dot( v, UnpackFactors );
}
vec2 packDepthToRG( in highp float v ) {
	return packDepthToRGBA( v ).yx;
}
float unpackRGToDepth( const in highp vec2 v ) {
	return unpackRGBAToDepth( vec4( v.xy, 0.0, 0.0 ) );
}
vec4 pack2HalfToRGBA( vec2 v ) {
	vec4 r = vec4( v.x, fract( v.x * 255.0 ), v.y, fract( v.y * 255.0 ) );
	return vec4( r.x - r.y / 255.0, r.y, r.z - r.w / 255.0, r.w );
}
vec2 unpackRGBATo2Half( vec4 v ) {
	return vec2( v.x + ( v.y / 255.0 ), v.z + ( v.w / 255.0 ) );
}
float viewZToOrthographicDepth( const in float viewZ, const in float near, const in float far ) {
	return ( viewZ + near ) / ( near - far );
}
float orthographicDepthToViewZ( const in float depth, const in float near, const in float far ) {
	return depth * ( near - far ) - near;
}
float viewZToPerspectiveDepth( const in float viewZ, const in float near, const in float far ) {
	return ( ( near + viewZ ) * far ) / ( ( far - near ) * viewZ );
}
float perspectiveDepthToViewZ( const in float depth, const in float near, const in float far ) {
	return ( near * far ) / ( ( far - near ) * depth - far );
}`,SM=`#ifdef PREMULTIPLIED_ALPHA
	gl_FragColor.rgb *= gl_FragColor.a;
#endif`,MM=`vec4 mvPosition = vec4( transformed, 1.0 );
#ifdef USE_BATCHING
	mvPosition = batchingMatrix * mvPosition;
#endif
#ifdef USE_INSTANCING
	mvPosition = instanceMatrix * mvPosition;
#endif
mvPosition = modelViewMatrix * mvPosition;
gl_Position = projectionMatrix * mvPosition;`,EM=`#ifdef DITHERING
	gl_FragColor.rgb = dithering( gl_FragColor.rgb );
#endif`,CM=`#ifdef DITHERING
	vec3 dithering( vec3 color ) {
		float grid_position = rand( gl_FragCoord.xy );
		vec3 dither_shift_RGB = vec3( 0.25 / 255.0, -0.25 / 255.0, 0.25 / 255.0 );
		dither_shift_RGB = mix( 2.0 * dither_shift_RGB, -2.0 * dither_shift_RGB, grid_position );
		return color + dither_shift_RGB;
	}
#endif`,TM=`float roughnessFactor = roughness;
#ifdef USE_ROUGHNESSMAP
	vec4 texelRoughness = texture2D( roughnessMap, vRoughnessMapUv );
	roughnessFactor *= texelRoughness.g;
#endif`,bM=`#ifdef USE_ROUGHNESSMAP
	uniform sampler2D roughnessMap;
#endif`,AM=`#if NUM_SPOT_LIGHT_COORDS > 0
	varying vec4 vSpotLightCoord[ NUM_SPOT_LIGHT_COORDS ];
#endif
#if NUM_SPOT_LIGHT_MAPS > 0
	uniform sampler2D spotLightMap[ NUM_SPOT_LIGHT_MAPS ];
#endif
#ifdef USE_SHADOWMAP
	#if NUM_DIR_LIGHT_SHADOWS > 0
		uniform sampler2D directionalShadowMap[ NUM_DIR_LIGHT_SHADOWS ];
		varying vec4 vDirectionalShadowCoord[ NUM_DIR_LIGHT_SHADOWS ];
		struct DirectionalLightShadow {
			float shadowIntensity;
			float shadowBias;
			float shadowNormalBias;
			float shadowRadius;
			vec2 shadowMapSize;
		};
		uniform DirectionalLightShadow directionalLightShadows[ NUM_DIR_LIGHT_SHADOWS ];
	#endif
	#if NUM_SPOT_LIGHT_SHADOWS > 0
		uniform sampler2D spotShadowMap[ NUM_SPOT_LIGHT_SHADOWS ];
		struct SpotLightShadow {
			float shadowIntensity;
			float shadowBias;
			float shadowNormalBias;
			float shadowRadius;
			vec2 shadowMapSize;
		};
		uniform SpotLightShadow spotLightShadows[ NUM_SPOT_LIGHT_SHADOWS ];
	#endif
	#if NUM_POINT_LIGHT_SHADOWS > 0
		uniform sampler2D pointShadowMap[ NUM_POINT_LIGHT_SHADOWS ];
		varying vec4 vPointShadowCoord[ NUM_POINT_LIGHT_SHADOWS ];
		struct PointLightShadow {
			float shadowIntensity;
			float shadowBias;
			float shadowNormalBias;
			float shadowRadius;
			vec2 shadowMapSize;
			float shadowCameraNear;
			float shadowCameraFar;
		};
		uniform PointLightShadow pointLightShadows[ NUM_POINT_LIGHT_SHADOWS ];
	#endif
	float texture2DCompare( sampler2D depths, vec2 uv, float compare ) {
		return step( compare, unpackRGBAToDepth( texture2D( depths, uv ) ) );
	}
	vec2 texture2DDistribution( sampler2D shadow, vec2 uv ) {
		return unpackRGBATo2Half( texture2D( shadow, uv ) );
	}
	float VSMShadow (sampler2D shadow, vec2 uv, float compare ){
		float occlusion = 1.0;
		vec2 distribution = texture2DDistribution( shadow, uv );
		float hard_shadow = step( compare , distribution.x );
		if (hard_shadow != 1.0 ) {
			float distance = compare - distribution.x ;
			float variance = max( 0.00000, distribution.y * distribution.y );
			float softness_probability = variance / (variance + distance * distance );			softness_probability = clamp( ( softness_probability - 0.3 ) / ( 0.95 - 0.3 ), 0.0, 1.0 );			occlusion = clamp( max( hard_shadow, softness_probability ), 0.0, 1.0 );
		}
		return occlusion;
	}
	float getShadow( sampler2D shadowMap, vec2 shadowMapSize, float shadowIntensity, float shadowBias, float shadowRadius, vec4 shadowCoord ) {
		float shadow = 1.0;
		shadowCoord.xyz /= shadowCoord.w;
		shadowCoord.z += shadowBias;
		bool inFrustum = shadowCoord.x >= 0.0 && shadowCoord.x <= 1.0 && shadowCoord.y >= 0.0 && shadowCoord.y <= 1.0;
		bool frustumTest = inFrustum && shadowCoord.z <= 1.0;
		if ( frustumTest ) {
		#if defined( SHADOWMAP_TYPE_PCF )
			vec2 texelSize = vec2( 1.0 ) / shadowMapSize;
			float dx0 = - texelSize.x * shadowRadius;
			float dy0 = - texelSize.y * shadowRadius;
			float dx1 = + texelSize.x * shadowRadius;
			float dy1 = + texelSize.y * shadowRadius;
			float dx2 = dx0 / 2.0;
			float dy2 = dy0 / 2.0;
			float dx3 = dx1 / 2.0;
			float dy3 = dy1 / 2.0;
			shadow = (
				texture2DCompare( shadowMap, shadowCoord.xy + vec2( dx0, dy0 ), shadowCoord.z ) +
				texture2DCompare( shadowMap, shadowCoord.xy + vec2( 0.0, dy0 ), shadowCoord.z ) +
				texture2DCompare( shadowMap, shadowCoord.xy + vec2( dx1, dy0 ), shadowCoord.z ) +
				texture2DCompare( shadowMap, shadowCoord.xy + vec2( dx2, dy2 ), shadowCoord.z ) +
				texture2DCompare( shadowMap, shadowCoord.xy + vec2( 0.0, dy2 ), shadowCoord.z ) +
				texture2DCompare( shadowMap, shadowCoord.xy + vec2( dx3, dy2 ), shadowCoord.z ) +
				texture2DCompare( shadowMap, shadowCoord.xy + vec2( dx0, 0.0 ), shadowCoord.z ) +
				texture2DCompare( shadowMap, shadowCoord.xy + vec2( dx2, 0.0 ), shadowCoord.z ) +
				texture2DCompare( shadowMap, shadowCoord.xy, shadowCoord.z ) +
				texture2DCompare( shadowMap, shadowCoord.xy + vec2( dx3, 0.0 ), shadowCoord.z ) +
				texture2DCompare( shadowMap, shadowCoord.xy + vec2( dx1, 0.0 ), shadowCoord.z ) +
				texture2DCompare( shadowMap, shadowCoord.xy + vec2( dx2, dy3 ), shadowCoord.z ) +
				texture2DCompare( shadowMap, shadowCoord.xy + vec2( 0.0, dy3 ), shadowCoord.z ) +
				texture2DCompare( shadowMap, shadowCoord.xy + vec2( dx3, dy3 ), shadowCoord.z ) +
				texture2DCompare( shadowMap, shadowCoord.xy + vec2( dx0, dy1 ), shadowCoord.z ) +
				texture2DCompare( shadowMap, shadowCoord.xy + vec2( 0.0, dy1 ), shadowCoord.z ) +
				texture2DCompare( shadowMap, shadowCoord.xy + vec2( dx1, dy1 ), shadowCoord.z )
			) * ( 1.0 / 17.0 );
		#elif defined( SHADOWMAP_TYPE_PCF_SOFT )
			vec2 texelSize = vec2( 1.0 ) / shadowMapSize;
			float dx = texelSize.x;
			float dy = texelSize.y;
			vec2 uv = shadowCoord.xy;
			vec2 f = fract( uv * shadowMapSize + 0.5 );
			uv -= f * texelSize;
			shadow = (
				texture2DCompare( shadowMap, uv, shadowCoord.z ) +
				texture2DCompare( shadowMap, uv + vec2( dx, 0.0 ), shadowCoord.z ) +
				texture2DCompare( shadowMap, uv + vec2( 0.0, dy ), shadowCoord.z ) +
				texture2DCompare( shadowMap, uv + texelSize, shadowCoord.z ) +
				mix( texture2DCompare( shadowMap, uv + vec2( -dx, 0.0 ), shadowCoord.z ),
					 texture2DCompare( shadowMap, uv + vec2( 2.0 * dx, 0.0 ), shadowCoord.z ),
					 f.x ) +
				mix( texture2DCompare( shadowMap, uv + vec2( -dx, dy ), shadowCoord.z ),
					 texture2DCompare( shadowMap, uv + vec2( 2.0 * dx, dy ), shadowCoord.z ),
					 f.x ) +
				mix( texture2DCompare( shadowMap, uv + vec2( 0.0, -dy ), shadowCoord.z ),
					 texture2DCompare( shadowMap, uv + vec2( 0.0, 2.0 * dy ), shadowCoord.z ),
					 f.y ) +
				mix( texture2DCompare( shadowMap, uv + vec2( dx, -dy ), shadowCoord.z ),
					 texture2DCompare( shadowMap, uv + vec2( dx, 2.0 * dy ), shadowCoord.z ),
					 f.y ) +
				mix( mix( texture2DCompare( shadowMap, uv + vec2( -dx, -dy ), shadowCoord.z ),
						  texture2DCompare( shadowMap, uv + vec2( 2.0 * dx, -dy ), shadowCoord.z ),
						  f.x ),
					 mix( texture2DCompare( shadowMap, uv + vec2( -dx, 2.0 * dy ), shadowCoord.z ),
						  texture2DCompare( shadowMap, uv + vec2( 2.0 * dx, 2.0 * dy ), shadowCoord.z ),
						  f.x ),
					 f.y )
			) * ( 1.0 / 9.0 );
		#elif defined( SHADOWMAP_TYPE_VSM )
			shadow = VSMShadow( shadowMap, shadowCoord.xy, shadowCoord.z );
		#else
			shadow = texture2DCompare( shadowMap, shadowCoord.xy, shadowCoord.z );
		#endif
		}
		return mix( 1.0, shadow, shadowIntensity );
	}
	vec2 cubeToUV( vec3 v, float texelSizeY ) {
		vec3 absV = abs( v );
		float scaleToCube = 1.0 / max( absV.x, max( absV.y, absV.z ) );
		absV *= scaleToCube;
		v *= scaleToCube * ( 1.0 - 2.0 * texelSizeY );
		vec2 planar = v.xy;
		float almostATexel = 1.5 * texelSizeY;
		float almostOne = 1.0 - almostATexel;
		if ( absV.z >= almostOne ) {
			if ( v.z > 0.0 )
				planar.x = 4.0 - v.x;
		} else if ( absV.x >= almostOne ) {
			float signX = sign( v.x );
			planar.x = v.z * signX + 2.0 * signX;
		} else if ( absV.y >= almostOne ) {
			float signY = sign( v.y );
			planar.x = v.x + 2.0 * signY + 2.0;
			planar.y = v.z * signY - 2.0;
		}
		return vec2( 0.125, 0.25 ) * planar + vec2( 0.375, 0.75 );
	}
	float getPointShadow( sampler2D shadowMap, vec2 shadowMapSize, float shadowIntensity, float shadowBias, float shadowRadius, vec4 shadowCoord, float shadowCameraNear, float shadowCameraFar ) {
		float shadow = 1.0;
		vec3 lightToPosition = shadowCoord.xyz;
		
		float lightToPositionLength = length( lightToPosition );
		if ( lightToPositionLength - shadowCameraFar <= 0.0 && lightToPositionLength - shadowCameraNear >= 0.0 ) {
			float dp = ( lightToPositionLength - shadowCameraNear ) / ( shadowCameraFar - shadowCameraNear );			dp += shadowBias;
			vec3 bd3D = normalize( lightToPosition );
			vec2 texelSize = vec2( 1.0 ) / ( shadowMapSize * vec2( 4.0, 2.0 ) );
			#if defined( SHADOWMAP_TYPE_PCF ) || defined( SHADOWMAP_TYPE_PCF_SOFT ) || defined( SHADOWMAP_TYPE_VSM )
				vec2 offset = vec2( - 1, 1 ) * shadowRadius * texelSize.y;
				shadow = (
					texture2DCompare( shadowMap, cubeToUV( bd3D + offset.xyy, texelSize.y ), dp ) +
					texture2DCompare( shadowMap, cubeToUV( bd3D + offset.yyy, texelSize.y ), dp ) +
					texture2DCompare( shadowMap, cubeToUV( bd3D + offset.xyx, texelSize.y ), dp ) +
					texture2DCompare( shadowMap, cubeToUV( bd3D + offset.yyx, texelSize.y ), dp ) +
					texture2DCompare( shadowMap, cubeToUV( bd3D, texelSize.y ), dp ) +
					texture2DCompare( shadowMap, cubeToUV( bd3D + offset.xxy, texelSize.y ), dp ) +
					texture2DCompare( shadowMap, cubeToUV( bd3D + offset.yxy, texelSize.y ), dp ) +
					texture2DCompare( shadowMap, cubeToUV( bd3D + offset.xxx, texelSize.y ), dp ) +
					texture2DCompare( shadowMap, cubeToUV( bd3D + offset.yxx, texelSize.y ), dp )
				) * ( 1.0 / 9.0 );
			#else
				shadow = texture2DCompare( shadowMap, cubeToUV( bd3D, texelSize.y ), dp );
			#endif
		}
		return mix( 1.0, shadow, shadowIntensity );
	}
#endif`,RM=`#if NUM_SPOT_LIGHT_COORDS > 0
	uniform mat4 spotLightMatrix[ NUM_SPOT_LIGHT_COORDS ];
	varying vec4 vSpotLightCoord[ NUM_SPOT_LIGHT_COORDS ];
#endif
#ifdef USE_SHADOWMAP
	#if NUM_DIR_LIGHT_SHADOWS > 0
		uniform mat4 directionalShadowMatrix[ NUM_DIR_LIGHT_SHADOWS ];
		varying vec4 vDirectionalShadowCoord[ NUM_DIR_LIGHT_SHADOWS ];
		struct DirectionalLightShadow {
			float shadowIntensity;
			float shadowBias;
			float shadowNormalBias;
			float shadowRadius;
			vec2 shadowMapSize;
		};
		uniform DirectionalLightShadow directionalLightShadows[ NUM_DIR_LIGHT_SHADOWS ];
	#endif
	#if NUM_SPOT_LIGHT_SHADOWS > 0
		struct SpotLightShadow {
			float shadowIntensity;
			float shadowBias;
			float shadowNormalBias;
			float shadowRadius;
			vec2 shadowMapSize;
		};
		uniform SpotLightShadow spotLightShadows[ NUM_SPOT_LIGHT_SHADOWS ];
	#endif
	#if NUM_POINT_LIGHT_SHADOWS > 0
		uniform mat4 pointShadowMatrix[ NUM_POINT_LIGHT_SHADOWS ];
		varying vec4 vPointShadowCoord[ NUM_POINT_LIGHT_SHADOWS ];
		struct PointLightShadow {
			float shadowIntensity;
			float shadowBias;
			float shadowNormalBias;
			float shadowRadius;
			vec2 shadowMapSize;
			float shadowCameraNear;
			float shadowCameraFar;
		};
		uniform PointLightShadow pointLightShadows[ NUM_POINT_LIGHT_SHADOWS ];
	#endif
#endif`,PM=`#if ( defined( USE_SHADOWMAP ) && ( NUM_DIR_LIGHT_SHADOWS > 0 || NUM_POINT_LIGHT_SHADOWS > 0 ) ) || ( NUM_SPOT_LIGHT_COORDS > 0 )
	vec3 shadowWorldNormal = inverseTransformDirection( transformedNormal, viewMatrix );
	vec4 shadowWorldPosition;
#endif
#if defined( USE_SHADOWMAP )
	#if NUM_DIR_LIGHT_SHADOWS > 0
		#pragma unroll_loop_start
		for ( int i = 0; i < NUM_DIR_LIGHT_SHADOWS; i ++ ) {
			shadowWorldPosition = worldPosition + vec4( shadowWorldNormal * directionalLightShadows[ i ].shadowNormalBias, 0 );
			vDirectionalShadowCoord[ i ] = directionalShadowMatrix[ i ] * shadowWorldPosition;
		}
		#pragma unroll_loop_end
	#endif
	#if NUM_POINT_LIGHT_SHADOWS > 0
		#pragma unroll_loop_start
		for ( int i = 0; i < NUM_POINT_LIGHT_SHADOWS; i ++ ) {
			shadowWorldPosition = worldPosition + vec4( shadowWorldNormal * pointLightShadows[ i ].shadowNormalBias, 0 );
			vPointShadowCoord[ i ] = pointShadowMatrix[ i ] * shadowWorldPosition;
		}
		#pragma unroll_loop_end
	#endif
#endif
#if NUM_SPOT_LIGHT_COORDS > 0
	#pragma unroll_loop_start
	for ( int i = 0; i < NUM_SPOT_LIGHT_COORDS; i ++ ) {
		shadowWorldPosition = worldPosition;
		#if ( defined( USE_SHADOWMAP ) && UNROLLED_LOOP_INDEX < NUM_SPOT_LIGHT_SHADOWS )
			shadowWorldPosition.xyz += shadowWorldNormal * spotLightShadows[ i ].shadowNormalBias;
		#endif
		vSpotLightCoord[ i ] = spotLightMatrix[ i ] * shadowWorldPosition;
	}
	#pragma unroll_loop_end
#endif`,IM=`float getShadowMask() {
	float shadow = 1.0;
	#ifdef USE_SHADOWMAP
	#if NUM_DIR_LIGHT_SHADOWS > 0
	DirectionalLightShadow directionalLight;
	#pragma unroll_loop_start
	for ( int i = 0; i < NUM_DIR_LIGHT_SHADOWS; i ++ ) {
		directionalLight = directionalLightShadows[ i ];
		shadow *= receiveShadow ? getShadow( directionalShadowMap[ i ], directionalLight.shadowMapSize, directionalLight.shadowIntensity, directionalLight.shadowBias, directionalLight.shadowRadius, vDirectionalShadowCoord[ i ] ) : 1.0;
	}
	#pragma unroll_loop_end
	#endif
	#if NUM_SPOT_LIGHT_SHADOWS > 0
	SpotLightShadow spotLight;
	#pragma unroll_loop_start
	for ( int i = 0; i < NUM_SPOT_LIGHT_SHADOWS; i ++ ) {
		spotLight = spotLightShadows[ i ];
		shadow *= receiveShadow ? getShadow( spotShadowMap[ i ], spotLight.shadowMapSize, spotLight.shadowIntensity, spotLight.shadowBias, spotLight.shadowRadius, vSpotLightCoord[ i ] ) : 1.0;
	}
	#pragma unroll_loop_end
	#endif
	#if NUM_POINT_LIGHT_SHADOWS > 0
	PointLightShadow pointLight;
	#pragma unroll_loop_start
	for ( int i = 0; i < NUM_POINT_LIGHT_SHADOWS; i ++ ) {
		pointLight = pointLightShadows[ i ];
		shadow *= receiveShadow ? getPointShadow( pointShadowMap[ i ], pointLight.shadowMapSize, pointLight.shadowIntensity, pointLight.shadowBias, pointLight.shadowRadius, vPointShadowCoord[ i ], pointLight.shadowCameraNear, pointLight.shadowCameraFar ) : 1.0;
	}
	#pragma unroll_loop_end
	#endif
	#endif
	return shadow;
}`,LM=`#ifdef USE_SKINNING
	mat4 boneMatX = getBoneMatrix( skinIndex.x );
	mat4 boneMatY = getBoneMatrix( skinIndex.y );
	mat4 boneMatZ = getBoneMatrix( skinIndex.z );
	mat4 boneMatW = getBoneMatrix( skinIndex.w );
#endif`,UM=`#ifdef USE_SKINNING
	uniform mat4 bindMatrix;
	uniform mat4 bindMatrixInverse;
	uniform highp sampler2D boneTexture;
	mat4 getBoneMatrix( const in float i ) {
		int size = textureSize( boneTexture, 0 ).x;
		int j = int( i ) * 4;
		int x = j % size;
		int y = j / size;
		vec4 v1 = texelFetch( boneTexture, ivec2( x, y ), 0 );
		vec4 v2 = texelFetch( boneTexture, ivec2( x + 1, y ), 0 );
		vec4 v3 = texelFetch( boneTexture, ivec2( x + 2, y ), 0 );
		vec4 v4 = texelFetch( boneTexture, ivec2( x + 3, y ), 0 );
		return mat4( v1, v2, v3, v4 );
	}
#endif`,NM=`#ifdef USE_SKINNING
	vec4 skinVertex = bindMatrix * vec4( transformed, 1.0 );
	vec4 skinned = vec4( 0.0 );
	skinned += boneMatX * skinVertex * skinWeight.x;
	skinned += boneMatY * skinVertex * skinWeight.y;
	skinned += boneMatZ * skinVertex * skinWeight.z;
	skinned += boneMatW * skinVertex * skinWeight.w;
	transformed = ( bindMatrixInverse * skinned ).xyz;
#endif`,DM=`#ifdef USE_SKINNING
	mat4 skinMatrix = mat4( 0.0 );
	skinMatrix += skinWeight.x * boneMatX;
	skinMatrix += skinWeight.y * boneMatY;
	skinMatrix += skinWeight.z * boneMatZ;
	skinMatrix += skinWeight.w * boneMatW;
	skinMatrix = bindMatrixInverse * skinMatrix * bindMatrix;
	objectNormal = vec4( skinMatrix * vec4( objectNormal, 0.0 ) ).xyz;
	#ifdef USE_TANGENT
		objectTangent = vec4( skinMatrix * vec4( objectTangent, 0.0 ) ).xyz;
	#endif
#endif`,OM=`float specularStrength;
#ifdef USE_SPECULARMAP
	vec4 texelSpecular = texture2D( specularMap, vSpecularMapUv );
	specularStrength = texelSpecular.r;
#else
	specularStrength = 1.0;
#endif`,FM=`#ifdef USE_SPECULARMAP
	uniform sampler2D specularMap;
#endif`,BM=`#if defined( TONE_MAPPING )
	gl_FragColor.rgb = toneMapping( gl_FragColor.rgb );
#endif`,kM=`#ifndef saturate
#define saturate( a ) clamp( a, 0.0, 1.0 )
#endif
uniform float toneMappingExposure;
vec3 LinearToneMapping( vec3 color ) {
	return saturate( toneMappingExposure * color );
}
vec3 ReinhardToneMapping( vec3 color ) {
	color *= toneMappingExposure;
	return saturate( color / ( vec3( 1.0 ) + color ) );
}
vec3 OptimizedCineonToneMapping( vec3 color ) {
	color *= toneMappingExposure;
	color = max( vec3( 0.0 ), color - 0.004 );
	return pow( ( color * ( 6.2 * color + 0.5 ) ) / ( color * ( 6.2 * color + 1.7 ) + 0.06 ), vec3( 2.2 ) );
}
vec3 RRTAndODTFit( vec3 v ) {
	vec3 a = v * ( v + 0.0245786 ) - 0.000090537;
	vec3 b = v * ( 0.983729 * v + 0.4329510 ) + 0.238081;
	return a / b;
}
vec3 ACESFilmicToneMapping( vec3 color ) {
	const mat3 ACESInputMat = mat3(
		vec3( 0.59719, 0.07600, 0.02840 ),		vec3( 0.35458, 0.90834, 0.13383 ),
		vec3( 0.04823, 0.01566, 0.83777 )
	);
	const mat3 ACESOutputMat = mat3(
		vec3(  1.60475, -0.10208, -0.00327 ),		vec3( -0.53108,  1.10813, -0.07276 ),
		vec3( -0.07367, -0.00605,  1.07602 )
	);
	color *= toneMappingExposure / 0.6;
	color = ACESInputMat * color;
	color = RRTAndODTFit( color );
	color = ACESOutputMat * color;
	return saturate( color );
}
const mat3 LINEAR_REC2020_TO_LINEAR_SRGB = mat3(
	vec3( 1.6605, - 0.1246, - 0.0182 ),
	vec3( - 0.5876, 1.1329, - 0.1006 ),
	vec3( - 0.0728, - 0.0083, 1.1187 )
);
const mat3 LINEAR_SRGB_TO_LINEAR_REC2020 = mat3(
	vec3( 0.6274, 0.0691, 0.0164 ),
	vec3( 0.3293, 0.9195, 0.0880 ),
	vec3( 0.0433, 0.0113, 0.8956 )
);
vec3 agxDefaultContrastApprox( vec3 x ) {
	vec3 x2 = x * x;
	vec3 x4 = x2 * x2;
	return + 15.5 * x4 * x2
		- 40.14 * x4 * x
		+ 31.96 * x4
		- 6.868 * x2 * x
		+ 0.4298 * x2
		+ 0.1191 * x
		- 0.00232;
}
vec3 AgXToneMapping( vec3 color ) {
	const mat3 AgXInsetMatrix = mat3(
		vec3( 0.856627153315983, 0.137318972929847, 0.11189821299995 ),
		vec3( 0.0951212405381588, 0.761241990602591, 0.0767994186031903 ),
		vec3( 0.0482516061458583, 0.101439036467562, 0.811302368396859 )
	);
	const mat3 AgXOutsetMatrix = mat3(
		vec3( 1.1271005818144368, - 0.1413297634984383, - 0.14132976349843826 ),
		vec3( - 0.11060664309660323, 1.157823702216272, - 0.11060664309660294 ),
		vec3( - 0.016493938717834573, - 0.016493938717834257, 1.2519364065950405 )
	);
	const float AgxMinEv = - 12.47393;	const float AgxMaxEv = 4.026069;
	color *= toneMappingExposure;
	color = LINEAR_SRGB_TO_LINEAR_REC2020 * color;
	color = AgXInsetMatrix * color;
	color = max( color, 1e-10 );	color = log2( color );
	color = ( color - AgxMinEv ) / ( AgxMaxEv - AgxMinEv );
	color = clamp( color, 0.0, 1.0 );
	color = agxDefaultContrastApprox( color );
	color = AgXOutsetMatrix * color;
	color = pow( max( vec3( 0.0 ), color ), vec3( 2.2 ) );
	color = LINEAR_REC2020_TO_LINEAR_SRGB * color;
	color = clamp( color, 0.0, 1.0 );
	return color;
}
vec3 NeutralToneMapping( vec3 color ) {
	const float StartCompression = 0.8 - 0.04;
	const float Desaturation = 0.15;
	color *= toneMappingExposure;
	float x = min( color.r, min( color.g, color.b ) );
	float offset = x < 0.08 ? x - 6.25 * x * x : 0.04;
	color -= offset;
	float peak = max( color.r, max( color.g, color.b ) );
	if ( peak < StartCompression ) return color;
	float d = 1. - StartCompression;
	float newPeak = 1. - d * d / ( peak + d - StartCompression );
	color *= newPeak / peak;
	float g = 1. - 1. / ( Desaturation * ( peak - newPeak ) + 1. );
	return mix( color, vec3( newPeak ), g );
}
vec3 CustomToneMapping( vec3 color ) { return color; }`,zM=`#ifdef USE_TRANSMISSION
	material.transmission = transmission;
	material.transmissionAlpha = 1.0;
	material.thickness = thickness;
	material.attenuationDistance = attenuationDistance;
	material.attenuationColor = attenuationColor;
	#ifdef USE_TRANSMISSIONMAP
		material.transmission *= texture2D( transmissionMap, vTransmissionMapUv ).r;
	#endif
	#ifdef USE_THICKNESSMAP
		material.thickness *= texture2D( thicknessMap, vThicknessMapUv ).g;
	#endif
	vec3 pos = vWorldPosition;
	vec3 v = normalize( cameraPosition - pos );
	vec3 n = inverseTransformDirection( normal, viewMatrix );
	vec4 transmitted = getIBLVolumeRefraction(
		n, v, material.roughness, material.diffuseColor, material.specularColor, material.specularF90,
		pos, modelMatrix, viewMatrix, projectionMatrix, material.dispersion, material.ior, material.thickness,
		material.attenuationColor, material.attenuationDistance );
	material.transmissionAlpha = mix( material.transmissionAlpha, transmitted.a, material.transmission );
	totalDiffuse = mix( totalDiffuse, transmitted.rgb, material.transmission );
#endif`,VM=`#ifdef USE_TRANSMISSION
	uniform float transmission;
	uniform float thickness;
	uniform float attenuationDistance;
	uniform vec3 attenuationColor;
	#ifdef USE_TRANSMISSIONMAP
		uniform sampler2D transmissionMap;
	#endif
	#ifdef USE_THICKNESSMAP
		uniform sampler2D thicknessMap;
	#endif
	uniform vec2 transmissionSamplerSize;
	uniform sampler2D transmissionSamplerMap;
	uniform mat4 modelMatrix;
	uniform mat4 projectionMatrix;
	varying vec3 vWorldPosition;
	float w0( float a ) {
		return ( 1.0 / 6.0 ) * ( a * ( a * ( - a + 3.0 ) - 3.0 ) + 1.0 );
	}
	float w1( float a ) {
		return ( 1.0 / 6.0 ) * ( a *  a * ( 3.0 * a - 6.0 ) + 4.0 );
	}
	float w2( float a ){
		return ( 1.0 / 6.0 ) * ( a * ( a * ( - 3.0 * a + 3.0 ) + 3.0 ) + 1.0 );
	}
	float w3( float a ) {
		return ( 1.0 / 6.0 ) * ( a * a * a );
	}
	float g0( float a ) {
		return w0( a ) + w1( a );
	}
	float g1( float a ) {
		return w2( a ) + w3( a );
	}
	float h0( float a ) {
		return - 1.0 + w1( a ) / ( w0( a ) + w1( a ) );
	}
	float h1( float a ) {
		return 1.0 + w3( a ) / ( w2( a ) + w3( a ) );
	}
	vec4 bicubic( sampler2D tex, vec2 uv, vec4 texelSize, float lod ) {
		uv = uv * texelSize.zw + 0.5;
		vec2 iuv = floor( uv );
		vec2 fuv = fract( uv );
		float g0x = g0( fuv.x );
		float g1x = g1( fuv.x );
		float h0x = h0( fuv.x );
		float h1x = h1( fuv.x );
		float h0y = h0( fuv.y );
		float h1y = h1( fuv.y );
		vec2 p0 = ( vec2( iuv.x + h0x, iuv.y + h0y ) - 0.5 ) * texelSize.xy;
		vec2 p1 = ( vec2( iuv.x + h1x, iuv.y + h0y ) - 0.5 ) * texelSize.xy;
		vec2 p2 = ( vec2( iuv.x + h0x, iuv.y + h1y ) - 0.5 ) * texelSize.xy;
		vec2 p3 = ( vec2( iuv.x + h1x, iuv.y + h1y ) - 0.5 ) * texelSize.xy;
		return g0( fuv.y ) * ( g0x * textureLod( tex, p0, lod ) + g1x * textureLod( tex, p1, lod ) ) +
			g1( fuv.y ) * ( g0x * textureLod( tex, p2, lod ) + g1x * textureLod( tex, p3, lod ) );
	}
	vec4 textureBicubic( sampler2D sampler, vec2 uv, float lod ) {
		vec2 fLodSize = vec2( textureSize( sampler, int( lod ) ) );
		vec2 cLodSize = vec2( textureSize( sampler, int( lod + 1.0 ) ) );
		vec2 fLodSizeInv = 1.0 / fLodSize;
		vec2 cLodSizeInv = 1.0 / cLodSize;
		vec4 fSample = bicubic( sampler, uv, vec4( fLodSizeInv, fLodSize ), floor( lod ) );
		vec4 cSample = bicubic( sampler, uv, vec4( cLodSizeInv, cLodSize ), ceil( lod ) );
		return mix( fSample, cSample, fract( lod ) );
	}
	vec3 getVolumeTransmissionRay( const in vec3 n, const in vec3 v, const in float thickness, const in float ior, const in mat4 modelMatrix ) {
		vec3 refractionVector = refract( - v, normalize( n ), 1.0 / ior );
		vec3 modelScale;
		modelScale.x = length( vec3( modelMatrix[ 0 ].xyz ) );
		modelScale.y = length( vec3( modelMatrix[ 1 ].xyz ) );
		modelScale.z = length( vec3( modelMatrix[ 2 ].xyz ) );
		return normalize( refractionVector ) * thickness * modelScale;
	}
	float applyIorToRoughness( const in float roughness, const in float ior ) {
		return roughness * clamp( ior * 2.0 - 2.0, 0.0, 1.0 );
	}
	vec4 getTransmissionSample( const in vec2 fragCoord, const in float roughness, const in float ior ) {
		float lod = log2( transmissionSamplerSize.x ) * applyIorToRoughness( roughness, ior );
		return textureBicubic( transmissionSamplerMap, fragCoord.xy, lod );
	}
	vec3 volumeAttenuation( const in float transmissionDistance, const in vec3 attenuationColor, const in float attenuationDistance ) {
		if ( isinf( attenuationDistance ) ) {
			return vec3( 1.0 );
		} else {
			vec3 attenuationCoefficient = -log( attenuationColor ) / attenuationDistance;
			vec3 transmittance = exp( - attenuationCoefficient * transmissionDistance );			return transmittance;
		}
	}
	vec4 getIBLVolumeRefraction( const in vec3 n, const in vec3 v, const in float roughness, const in vec3 diffuseColor,
		const in vec3 specularColor, const in float specularF90, const in vec3 position, const in mat4 modelMatrix,
		const in mat4 viewMatrix, const in mat4 projMatrix, const in float dispersion, const in float ior, const in float thickness,
		const in vec3 attenuationColor, const in float attenuationDistance ) {
		vec4 transmittedLight;
		vec3 transmittance;
		#ifdef USE_DISPERSION
			float halfSpread = ( ior - 1.0 ) * 0.025 * dispersion;
			vec3 iors = vec3( ior - halfSpread, ior, ior + halfSpread );
			for ( int i = 0; i < 3; i ++ ) {
				vec3 transmissionRay = getVolumeTransmissionRay( n, v, thickness, iors[ i ], modelMatrix );
				vec3 refractedRayExit = position + transmissionRay;
		
				vec4 ndcPos = projMatrix * viewMatrix * vec4( refractedRayExit, 1.0 );
				vec2 refractionCoords = ndcPos.xy / ndcPos.w;
				refractionCoords += 1.0;
				refractionCoords /= 2.0;
		
				vec4 transmissionSample = getTransmissionSample( refractionCoords, roughness, iors[ i ] );
				transmittedLight[ i ] = transmissionSample[ i ];
				transmittedLight.a += transmissionSample.a;
				transmittance[ i ] = diffuseColor[ i ] * volumeAttenuation( length( transmissionRay ), attenuationColor, attenuationDistance )[ i ];
			}
			transmittedLight.a /= 3.0;
		
		#else
		
			vec3 transmissionRay = getVolumeTransmissionRay( n, v, thickness, ior, modelMatrix );
			vec3 refractedRayExit = position + transmissionRay;
			vec4 ndcPos = projMatrix * viewMatrix * vec4( refractedRayExit, 1.0 );
			vec2 refractionCoords = ndcPos.xy / ndcPos.w;
			refractionCoords += 1.0;
			refractionCoords /= 2.0;
			transmittedLight = getTransmissionSample( refractionCoords, roughness, ior );
			transmittance = diffuseColor * volumeAttenuation( length( transmissionRay ), attenuationColor, attenuationDistance );
		
		#endif
		vec3 attenuatedColor = transmittance * transmittedLight.rgb;
		vec3 F = EnvironmentBRDF( n, v, specularColor, specularF90, roughness );
		float transmittanceFactor = ( transmittance.r + transmittance.g + transmittance.b ) / 3.0;
		return vec4( ( 1.0 - F ) * attenuatedColor, 1.0 - ( 1.0 - transmittedLight.a ) * transmittanceFactor );
	}
#endif`,HM=`#if defined( USE_UV ) || defined( USE_ANISOTROPY )
	varying vec2 vUv;
#endif
#ifdef USE_MAP
	varying vec2 vMapUv;
#endif
#ifdef USE_ALPHAMAP
	varying vec2 vAlphaMapUv;
#endif
#ifdef USE_LIGHTMAP
	varying vec2 vLightMapUv;
#endif
#ifdef USE_AOMAP
	varying vec2 vAoMapUv;
#endif
#ifdef USE_BUMPMAP
	varying vec2 vBumpMapUv;
#endif
#ifdef USE_NORMALMAP
	varying vec2 vNormalMapUv;
#endif
#ifdef USE_EMISSIVEMAP
	varying vec2 vEmissiveMapUv;
#endif
#ifdef USE_METALNESSMAP
	varying vec2 vMetalnessMapUv;
#endif
#ifdef USE_ROUGHNESSMAP
	varying vec2 vRoughnessMapUv;
#endif
#ifdef USE_ANISOTROPYMAP
	varying vec2 vAnisotropyMapUv;
#endif
#ifdef USE_CLEARCOATMAP
	varying vec2 vClearcoatMapUv;
#endif
#ifdef USE_CLEARCOAT_NORMALMAP
	varying vec2 vClearcoatNormalMapUv;
#endif
#ifdef USE_CLEARCOAT_ROUGHNESSMAP
	varying vec2 vClearcoatRoughnessMapUv;
#endif
#ifdef USE_IRIDESCENCEMAP
	varying vec2 vIridescenceMapUv;
#endif
#ifdef USE_IRIDESCENCE_THICKNESSMAP
	varying vec2 vIridescenceThicknessMapUv;
#endif
#ifdef USE_SHEEN_COLORMAP
	varying vec2 vSheenColorMapUv;
#endif
#ifdef USE_SHEEN_ROUGHNESSMAP
	varying vec2 vSheenRoughnessMapUv;
#endif
#ifdef USE_SPECULARMAP
	varying vec2 vSpecularMapUv;
#endif
#ifdef USE_SPECULAR_COLORMAP
	varying vec2 vSpecularColorMapUv;
#endif
#ifdef USE_SPECULAR_INTENSITYMAP
	varying vec2 vSpecularIntensityMapUv;
#endif
#ifdef USE_TRANSMISSIONMAP
	uniform mat3 transmissionMapTransform;
	varying vec2 vTransmissionMapUv;
#endif
#ifdef USE_THICKNESSMAP
	uniform mat3 thicknessMapTransform;
	varying vec2 vThicknessMapUv;
#endif`,GM=`#if defined( USE_UV ) || defined( USE_ANISOTROPY )
	varying vec2 vUv;
#endif
#ifdef USE_MAP
	uniform mat3 mapTransform;
	varying vec2 vMapUv;
#endif
#ifdef USE_ALPHAMAP
	uniform mat3 alphaMapTransform;
	varying vec2 vAlphaMapUv;
#endif
#ifdef USE_LIGHTMAP
	uniform mat3 lightMapTransform;
	varying vec2 vLightMapUv;
#endif
#ifdef USE_AOMAP
	uniform mat3 aoMapTransform;
	varying vec2 vAoMapUv;
#endif
#ifdef USE_BUMPMAP
	uniform mat3 bumpMapTransform;
	varying vec2 vBumpMapUv;
#endif
#ifdef USE_NORMALMAP
	uniform mat3 normalMapTransform;
	varying vec2 vNormalMapUv;
#endif
#ifdef USE_DISPLACEMENTMAP
	uniform mat3 displacementMapTransform;
	varying vec2 vDisplacementMapUv;
#endif
#ifdef USE_EMISSIVEMAP
	uniform mat3 emissiveMapTransform;
	varying vec2 vEmissiveMapUv;
#endif
#ifdef USE_METALNESSMAP
	uniform mat3 metalnessMapTransform;
	varying vec2 vMetalnessMapUv;
#endif
#ifdef USE_ROUGHNESSMAP
	uniform mat3 roughnessMapTransform;
	varying vec2 vRoughnessMapUv;
#endif
#ifdef USE_ANISOTROPYMAP
	uniform mat3 anisotropyMapTransform;
	varying vec2 vAnisotropyMapUv;
#endif
#ifdef USE_CLEARCOATMAP
	uniform mat3 clearcoatMapTransform;
	varying vec2 vClearcoatMapUv;
#endif
#ifdef USE_CLEARCOAT_NORMALMAP
	uniform mat3 clearcoatNormalMapTransform;
	varying vec2 vClearcoatNormalMapUv;
#endif
#ifdef USE_CLEARCOAT_ROUGHNESSMAP
	uniform mat3 clearcoatRoughnessMapTransform;
	varying vec2 vClearcoatRoughnessMapUv;
#endif
#ifdef USE_SHEEN_COLORMAP
	uniform mat3 sheenColorMapTransform;
	varying vec2 vSheenColorMapUv;
#endif
#ifdef USE_SHEEN_ROUGHNESSMAP
	uniform mat3 sheenRoughnessMapTransform;
	varying vec2 vSheenRoughnessMapUv;
#endif
#ifdef USE_IRIDESCENCEMAP
	uniform mat3 iridescenceMapTransform;
	varying vec2 vIridescenceMapUv;
#endif
#ifdef USE_IRIDESCENCE_THICKNESSMAP
	uniform mat3 iridescenceThicknessMapTransform;
	varying vec2 vIridescenceThicknessMapUv;
#endif
#ifdef USE_SPECULARMAP
	uniform mat3 specularMapTransform;
	varying vec2 vSpecularMapUv;
#endif
#ifdef USE_SPECULAR_COLORMAP
	uniform mat3 specularColorMapTransform;
	varying vec2 vSpecularColorMapUv;
#endif
#ifdef USE_SPECULAR_INTENSITYMAP
	uniform mat3 specularIntensityMapTransform;
	varying vec2 vSpecularIntensityMapUv;
#endif
#ifdef USE_TRANSMISSIONMAP
	uniform mat3 transmissionMapTransform;
	varying vec2 vTransmissionMapUv;
#endif
#ifdef USE_THICKNESSMAP
	uniform mat3 thicknessMapTransform;
	varying vec2 vThicknessMapUv;
#endif`,WM=`#if defined( USE_UV ) || defined( USE_ANISOTROPY )
	vUv = vec3( uv, 1 ).xy;
#endif
#ifdef USE_MAP
	vMapUv = ( mapTransform * vec3( MAP_UV, 1 ) ).xy;
#endif
#ifdef USE_ALPHAMAP
	vAlphaMapUv = ( alphaMapTransform * vec3( ALPHAMAP_UV, 1 ) ).xy;
#endif
#ifdef USE_LIGHTMAP
	vLightMapUv = ( lightMapTransform * vec3( LIGHTMAP_UV, 1 ) ).xy;
#endif
#ifdef USE_AOMAP
	vAoMapUv = ( aoMapTransform * vec3( AOMAP_UV, 1 ) ).xy;
#endif
#ifdef USE_BUMPMAP
	vBumpMapUv = ( bumpMapTransform * vec3( BUMPMAP_UV, 1 ) ).xy;
#endif
#ifdef USE_NORMALMAP
	vNormalMapUv = ( normalMapTransform * vec3( NORMALMAP_UV, 1 ) ).xy;
#endif
#ifdef USE_DISPLACEMENTMAP
	vDisplacementMapUv = ( displacementMapTransform * vec3( DISPLACEMENTMAP_UV, 1 ) ).xy;
#endif
#ifdef USE_EMISSIVEMAP
	vEmissiveMapUv = ( emissiveMapTransform * vec3( EMISSIVEMAP_UV, 1 ) ).xy;
#endif
#ifdef USE_METALNESSMAP
	vMetalnessMapUv = ( metalnessMapTransform * vec3( METALNESSMAP_UV, 1 ) ).xy;
#endif
#ifdef USE_ROUGHNESSMAP
	vRoughnessMapUv = ( roughnessMapTransform * vec3( ROUGHNESSMAP_UV, 1 ) ).xy;
#endif
#ifdef USE_ANISOTROPYMAP
	vAnisotropyMapUv = ( anisotropyMapTransform * vec3( ANISOTROPYMAP_UV, 1 ) ).xy;
#endif
#ifdef USE_CLEARCOATMAP
	vClearcoatMapUv = ( clearcoatMapTransform * vec3( CLEARCOATMAP_UV, 1 ) ).xy;
#endif
#ifdef USE_CLEARCOAT_NORMALMAP
	vClearcoatNormalMapUv = ( clearcoatNormalMapTransform * vec3( CLEARCOAT_NORMALMAP_UV, 1 ) ).xy;
#endif
#ifdef USE_CLEARCOAT_ROUGHNESSMAP
	vClearcoatRoughnessMapUv = ( clearcoatRoughnessMapTransform * vec3( CLEARCOAT_ROUGHNESSMAP_UV, 1 ) ).xy;
#endif
#ifdef USE_IRIDESCENCEMAP
	vIridescenceMapUv = ( iridescenceMapTransform * vec3( IRIDESCENCEMAP_UV, 1 ) ).xy;
#endif
#ifdef USE_IRIDESCENCE_THICKNESSMAP
	vIridescenceThicknessMapUv = ( iridescenceThicknessMapTransform * vec3( IRIDESCENCE_THICKNESSMAP_UV, 1 ) ).xy;
#endif
#ifdef USE_SHEEN_COLORMAP
	vSheenColorMapUv = ( sheenColorMapTransform * vec3( SHEEN_COLORMAP_UV, 1 ) ).xy;
#endif
#ifdef USE_SHEEN_ROUGHNESSMAP
	vSheenRoughnessMapUv = ( sheenRoughnessMapTransform * vec3( SHEEN_ROUGHNESSMAP_UV, 1 ) ).xy;
#endif
#ifdef USE_SPECULARMAP
	vSpecularMapUv = ( specularMapTransform * vec3( SPECULARMAP_UV, 1 ) ).xy;
#endif
#ifdef USE_SPECULAR_COLORMAP
	vSpecularColorMapUv = ( specularColorMapTransform * vec3( SPECULAR_COLORMAP_UV, 1 ) ).xy;
#endif
#ifdef USE_SPECULAR_INTENSITYMAP
	vSpecularIntensityMapUv = ( specularIntensityMapTransform * vec3( SPECULAR_INTENSITYMAP_UV, 1 ) ).xy;
#endif
#ifdef USE_TRANSMISSIONMAP
	vTransmissionMapUv = ( transmissionMapTransform * vec3( TRANSMISSIONMAP_UV, 1 ) ).xy;
#endif
#ifdef USE_THICKNESSMAP
	vThicknessMapUv = ( thicknessMapTransform * vec3( THICKNESSMAP_UV, 1 ) ).xy;
#endif`,XM=`#if defined( USE_ENVMAP ) || defined( DISTANCE ) || defined ( USE_SHADOWMAP ) || defined ( USE_TRANSMISSION ) || NUM_SPOT_LIGHT_COORDS > 0
	vec4 worldPosition = vec4( transformed, 1.0 );
	#ifdef USE_BATCHING
		worldPosition = batchingMatrix * worldPosition;
	#endif
	#ifdef USE_INSTANCING
		worldPosition = instanceMatrix * worldPosition;
	#endif
	worldPosition = modelMatrix * worldPosition;
#endif`,YM=`varying vec2 vUv;
uniform mat3 uvTransform;
void main() {
	vUv = ( uvTransform * vec3( uv, 1 ) ).xy;
	gl_Position = vec4( position.xy, 1.0, 1.0 );
}`,qM=`uniform sampler2D t2D;
uniform float backgroundIntensity;
varying vec2 vUv;
void main() {
	vec4 texColor = texture2D( t2D, vUv );
	#ifdef DECODE_VIDEO_TEXTURE
		texColor = vec4( mix( pow( texColor.rgb * 0.9478672986 + vec3( 0.0521327014 ), vec3( 2.4 ) ), texColor.rgb * 0.0773993808, vec3( lessThanEqual( texColor.rgb, vec3( 0.04045 ) ) ) ), texColor.w );
	#endif
	texColor.rgb *= backgroundIntensity;
	gl_FragColor = texColor;
	#include <tonemapping_fragment>
	#include <colorspace_fragment>
}`,$M=`varying vec3 vWorldDirection;
#include <common>
void main() {
	vWorldDirection = transformDirection( position, modelMatrix );
	#include <begin_vertex>
	#include <project_vertex>
	gl_Position.z = gl_Position.w;
}`,ZM=`#ifdef ENVMAP_TYPE_CUBE
	uniform samplerCube envMap;
#elif defined( ENVMAP_TYPE_CUBE_UV )
	uniform sampler2D envMap;
#endif
uniform float flipEnvMap;
uniform float backgroundBlurriness;
uniform float backgroundIntensity;
uniform mat3 backgroundRotation;
varying vec3 vWorldDirection;
#include <cube_uv_reflection_fragment>
void main() {
	#ifdef ENVMAP_TYPE_CUBE
		vec4 texColor = textureCube( envMap, backgroundRotation * vec3( flipEnvMap * vWorldDirection.x, vWorldDirection.yz ) );
	#elif defined( ENVMAP_TYPE_CUBE_UV )
		vec4 texColor = textureCubeUV( envMap, backgroundRotation * vWorldDirection, backgroundBlurriness );
	#else
		vec4 texColor = vec4( 0.0, 0.0, 0.0, 1.0 );
	#endif
	texColor.rgb *= backgroundIntensity;
	gl_FragColor = texColor;
	#include <tonemapping_fragment>
	#include <colorspace_fragment>
}`,JM=`varying vec3 vWorldDirection;
#include <common>
void main() {
	vWorldDirection = transformDirection( position, modelMatrix );
	#include <begin_vertex>
	#include <project_vertex>
	gl_Position.z = gl_Position.w;
}`,KM=`uniform samplerCube tCube;
uniform float tFlip;
uniform float opacity;
varying vec3 vWorldDirection;
void main() {
	vec4 texColor = textureCube( tCube, vec3( tFlip * vWorldDirection.x, vWorldDirection.yz ) );
	gl_FragColor = texColor;
	gl_FragColor.a *= opacity;
	#include <tonemapping_fragment>
	#include <colorspace_fragment>
}`,jM=`#include <common>
#include <batching_pars_vertex>
#include <uv_pars_vertex>
#include <displacementmap_pars_vertex>
#include <morphtarget_pars_vertex>
#include <skinning_pars_vertex>
#include <logdepthbuf_pars_vertex>
#include <clipping_planes_pars_vertex>
varying vec2 vHighPrecisionZW;
void main() {
	#include <uv_vertex>
	#include <batching_vertex>
	#include <skinbase_vertex>
	#include <morphinstance_vertex>
	#ifdef USE_DISPLACEMENTMAP
		#include <beginnormal_vertex>
		#include <morphnormal_vertex>
		#include <skinnormal_vertex>
	#endif
	#include <begin_vertex>
	#include <morphtarget_vertex>
	#include <skinning_vertex>
	#include <displacementmap_vertex>
	#include <project_vertex>
	#include <logdepthbuf_vertex>
	#include <clipping_planes_vertex>
	vHighPrecisionZW = gl_Position.zw;
}`,QM=`#if DEPTH_PACKING == 3200
	uniform float opacity;
#endif
#include <common>
#include <packing>
#include <uv_pars_fragment>
#include <map_pars_fragment>
#include <alphamap_pars_fragment>
#include <alphatest_pars_fragment>
#include <alphahash_pars_fragment>
#include <logdepthbuf_pars_fragment>
#include <clipping_planes_pars_fragment>
varying vec2 vHighPrecisionZW;
void main() {
	vec4 diffuseColor = vec4( 1.0 );
	#include <clipping_planes_fragment>
	#if DEPTH_PACKING == 3200
		diffuseColor.a = opacity;
	#endif
	#include <map_fragment>
	#include <alphamap_fragment>
	#include <alphatest_fragment>
	#include <alphahash_fragment>
	#include <logdepthbuf_fragment>
	float fragCoordZ = 0.5 * vHighPrecisionZW[0] / vHighPrecisionZW[1] + 0.5;
	#if DEPTH_PACKING == 3200
		gl_FragColor = vec4( vec3( 1.0 - fragCoordZ ), opacity );
	#elif DEPTH_PACKING == 3201
		gl_FragColor = packDepthToRGBA( fragCoordZ );
	#endif
}`,eE=`#define DISTANCE
varying vec3 vWorldPosition;
#include <common>
#include <batching_pars_vertex>
#include <uv_pars_vertex>
#include <displacementmap_pars_vertex>
#include <morphtarget_pars_vertex>
#include <skinning_pars_vertex>
#include <clipping_planes_pars_vertex>
void main() {
	#include <uv_vertex>
	#include <batching_vertex>
	#include <skinbase_vertex>
	#include <morphinstance_vertex>
	#ifdef USE_DISPLACEMENTMAP
		#include <beginnormal_vertex>
		#include <morphnormal_vertex>
		#include <skinnormal_vertex>
	#endif
	#include <begin_vertex>
	#include <morphtarget_vertex>
	#include <skinning_vertex>
	#include <displacementmap_vertex>
	#include <project_vertex>
	#include <worldpos_vertex>
	#include <clipping_planes_vertex>
	vWorldPosition = worldPosition.xyz;
}`,tE=`#define DISTANCE
uniform vec3 referencePosition;
uniform float nearDistance;
uniform float farDistance;
varying vec3 vWorldPosition;
#include <common>
#include <packing>
#include <uv_pars_fragment>
#include <map_pars_fragment>
#include <alphamap_pars_fragment>
#include <alphatest_pars_fragment>
#include <alphahash_pars_fragment>
#include <clipping_planes_pars_fragment>
void main () {
	vec4 diffuseColor = vec4( 1.0 );
	#include <clipping_planes_fragment>
	#include <map_fragment>
	#include <alphamap_fragment>
	#include <alphatest_fragment>
	#include <alphahash_fragment>
	float dist = length( vWorldPosition - referencePosition );
	dist = ( dist - nearDistance ) / ( farDistance - nearDistance );
	dist = saturate( dist );
	gl_FragColor = packDepthToRGBA( dist );
}`,nE=`varying vec3 vWorldDirection;
#include <common>
void main() {
	vWorldDirection = transformDirection( position, modelMatrix );
	#include <begin_vertex>
	#include <project_vertex>
}`,iE=`uniform sampler2D tEquirect;
varying vec3 vWorldDirection;
#include <common>
void main() {
	vec3 direction = normalize( vWorldDirection );
	vec2 sampleUV = equirectUv( direction );
	gl_FragColor = texture2D( tEquirect, sampleUV );
	#include <tonemapping_fragment>
	#include <colorspace_fragment>
}`,rE=`uniform float scale;
attribute float lineDistance;
varying float vLineDistance;
#include <common>
#include <uv_pars_vertex>
#include <color_pars_vertex>
#include <fog_pars_vertex>
#include <morphtarget_pars_vertex>
#include <logdepthbuf_pars_vertex>
#include <clipping_planes_pars_vertex>
void main() {
	vLineDistance = scale * lineDistance;
	#include <uv_vertex>
	#include <color_vertex>
	#include <morphinstance_vertex>
	#include <morphcolor_vertex>
	#include <begin_vertex>
	#include <morphtarget_vertex>
	#include <project_vertex>
	#include <logdepthbuf_vertex>
	#include <clipping_planes_vertex>
	#include <fog_vertex>
}`,sE=`uniform vec3 diffuse;
uniform float opacity;
uniform float dashSize;
uniform float totalSize;
varying float vLineDistance;
#include <common>
#include <color_pars_fragment>
#include <uv_pars_fragment>
#include <map_pars_fragment>
#include <fog_pars_fragment>
#include <logdepthbuf_pars_fragment>
#include <clipping_planes_pars_fragment>
void main() {
	vec4 diffuseColor = vec4( diffuse, opacity );
	#include <clipping_planes_fragment>
	if ( mod( vLineDistance, totalSize ) > dashSize ) {
		discard;
	}
	vec3 outgoingLight = vec3( 0.0 );
	#include <logdepthbuf_fragment>
	#include <map_fragment>
	#include <color_fragment>
	outgoingLight = diffuseColor.rgb;
	#include <opaque_fragment>
	#include <tonemapping_fragment>
	#include <colorspace_fragment>
	#include <fog_fragment>
	#include <premultiplied_alpha_fragment>
}`,oE=`#include <common>
#include <batching_pars_vertex>
#include <uv_pars_vertex>
#include <envmap_pars_vertex>
#include <color_pars_vertex>
#include <fog_pars_vertex>
#include <morphtarget_pars_vertex>
#include <skinning_pars_vertex>
#include <logdepthbuf_pars_vertex>
#include <clipping_planes_pars_vertex>
void main() {
	#include <uv_vertex>
	#include <color_vertex>
	#include <morphinstance_vertex>
	#include <morphcolor_vertex>
	#include <batching_vertex>
	#if defined ( USE_ENVMAP ) || defined ( USE_SKINNING )
		#include <beginnormal_vertex>
		#include <morphnormal_vertex>
		#include <skinbase_vertex>
		#include <skinnormal_vertex>
		#include <defaultnormal_vertex>
	#endif
	#include <begin_vertex>
	#include <morphtarget_vertex>
	#include <skinning_vertex>
	#include <project_vertex>
	#include <logdepthbuf_vertex>
	#include <clipping_planes_vertex>
	#include <worldpos_vertex>
	#include <envmap_vertex>
	#include <fog_vertex>
}`,aE=`uniform vec3 diffuse;
uniform float opacity;
#ifndef FLAT_SHADED
	varying vec3 vNormal;
#endif
#include <common>
#include <dithering_pars_fragment>
#include <color_pars_fragment>
#include <uv_pars_fragment>
#include <map_pars_fragment>
#include <alphamap_pars_fragment>
#include <alphatest_pars_fragment>
#include <alphahash_pars_fragment>
#include <aomap_pars_fragment>
#include <lightmap_pars_fragment>
#include <envmap_common_pars_fragment>
#include <envmap_pars_fragment>
#include <fog_pars_fragment>
#include <specularmap_pars_fragment>
#include <logdepthbuf_pars_fragment>
#include <clipping_planes_pars_fragment>
void main() {
	vec4 diffuseColor = vec4( diffuse, opacity );
	#include <clipping_planes_fragment>
	#include <logdepthbuf_fragment>
	#include <map_fragment>
	#include <color_fragment>
	#include <alphamap_fragment>
	#include <alphatest_fragment>
	#include <alphahash_fragment>
	#include <specularmap_fragment>
	ReflectedLight reflectedLight = ReflectedLight( vec3( 0.0 ), vec3( 0.0 ), vec3( 0.0 ), vec3( 0.0 ) );
	#ifdef USE_LIGHTMAP
		vec4 lightMapTexel = texture2D( lightMap, vLightMapUv );
		reflectedLight.indirectDiffuse += lightMapTexel.rgb * lightMapIntensity * RECIPROCAL_PI;
	#else
		reflectedLight.indirectDiffuse += vec3( 1.0 );
	#endif
	#include <aomap_fragment>
	reflectedLight.indirectDiffuse *= diffuseColor.rgb;
	vec3 outgoingLight = reflectedLight.indirectDiffuse;
	#include <envmap_fragment>
	#include <opaque_fragment>
	#include <tonemapping_fragment>
	#include <colorspace_fragment>
	#include <fog_fragment>
	#include <premultiplied_alpha_fragment>
	#include <dithering_fragment>
}`,lE=`#define LAMBERT
varying vec3 vViewPosition;
#include <common>
#include <batching_pars_vertex>
#include <uv_pars_vertex>
#include <displacementmap_pars_vertex>
#include <envmap_pars_vertex>
#include <color_pars_vertex>
#include <fog_pars_vertex>
#include <normal_pars_vertex>
#include <morphtarget_pars_vertex>
#include <skinning_pars_vertex>
#include <shadowmap_pars_vertex>
#include <logdepthbuf_pars_vertex>
#include <clipping_planes_pars_vertex>
void main() {
	#include <uv_vertex>
	#include <color_vertex>
	#include <morphinstance_vertex>
	#include <morphcolor_vertex>
	#include <batching_vertex>
	#include <beginnormal_vertex>
	#include <morphnormal_vertex>
	#include <skinbase_vertex>
	#include <skinnormal_vertex>
	#include <defaultnormal_vertex>
	#include <normal_vertex>
	#include <begin_vertex>
	#include <morphtarget_vertex>
	#include <skinning_vertex>
	#include <displacementmap_vertex>
	#include <project_vertex>
	#include <logdepthbuf_vertex>
	#include <clipping_planes_vertex>
	vViewPosition = - mvPosition.xyz;
	#include <worldpos_vertex>
	#include <envmap_vertex>
	#include <shadowmap_vertex>
	#include <fog_vertex>
}`,cE=`#define LAMBERT
uniform vec3 diffuse;
uniform vec3 emissive;
uniform float opacity;
#include <common>
#include <packing>
#include <dithering_pars_fragment>
#include <color_pars_fragment>
#include <uv_pars_fragment>
#include <map_pars_fragment>
#include <alphamap_pars_fragment>
#include <alphatest_pars_fragment>
#include <alphahash_pars_fragment>
#include <aomap_pars_fragment>
#include <lightmap_pars_fragment>
#include <emissivemap_pars_fragment>
#include <envmap_common_pars_fragment>
#include <envmap_pars_fragment>
#include <fog_pars_fragment>
#include <bsdfs>
#include <lights_pars_begin>
#include <normal_pars_fragment>
#include <lights_lambert_pars_fragment>
#include <shadowmap_pars_fragment>
#include <bumpmap_pars_fragment>
#include <normalmap_pars_fragment>
#include <specularmap_pars_fragment>
#include <logdepthbuf_pars_fragment>
#include <clipping_planes_pars_fragment>
void main() {
	vec4 diffuseColor = vec4( diffuse, opacity );
	#include <clipping_planes_fragment>
	ReflectedLight reflectedLight = ReflectedLight( vec3( 0.0 ), vec3( 0.0 ), vec3( 0.0 ), vec3( 0.0 ) );
	vec3 totalEmissiveRadiance = emissive;
	#include <logdepthbuf_fragment>
	#include <map_fragment>
	#include <color_fragment>
	#include <alphamap_fragment>
	#include <alphatest_fragment>
	#include <alphahash_fragment>
	#include <specularmap_fragment>
	#include <normal_fragment_begin>
	#include <normal_fragment_maps>
	#include <emissivemap_fragment>
	#include <lights_lambert_fragment>
	#include <lights_fragment_begin>
	#include <lights_fragment_maps>
	#include <lights_fragment_end>
	#include <aomap_fragment>
	vec3 outgoingLight = reflectedLight.directDiffuse + reflectedLight.indirectDiffuse + totalEmissiveRadiance;
	#include <envmap_fragment>
	#include <opaque_fragment>
	#include <tonemapping_fragment>
	#include <colorspace_fragment>
	#include <fog_fragment>
	#include <premultiplied_alpha_fragment>
	#include <dithering_fragment>
}`,uE=`#define MATCAP
varying vec3 vViewPosition;
#include <common>
#include <batching_pars_vertex>
#include <uv_pars_vertex>
#include <color_pars_vertex>
#include <displacementmap_pars_vertex>
#include <fog_pars_vertex>
#include <normal_pars_vertex>
#include <morphtarget_pars_vertex>
#include <skinning_pars_vertex>
#include <logdepthbuf_pars_vertex>
#include <clipping_planes_pars_vertex>
void main() {
	#include <uv_vertex>
	#include <color_vertex>
	#include <morphinstance_vertex>
	#include <morphcolor_vertex>
	#include <batching_vertex>
	#include <beginnormal_vertex>
	#include <morphnormal_vertex>
	#include <skinbase_vertex>
	#include <skinnormal_vertex>
	#include <defaultnormal_vertex>
	#include <normal_vertex>
	#include <begin_vertex>
	#include <morphtarget_vertex>
	#include <skinning_vertex>
	#include <displacementmap_vertex>
	#include <project_vertex>
	#include <logdepthbuf_vertex>
	#include <clipping_planes_vertex>
	#include <fog_vertex>
	vViewPosition = - mvPosition.xyz;
}`,hE=`#define MATCAP
uniform vec3 diffuse;
uniform float opacity;
uniform sampler2D matcap;
varying vec3 vViewPosition;
#include <common>
#include <dithering_pars_fragment>
#include <color_pars_fragment>
#include <uv_pars_fragment>
#include <map_pars_fragment>
#include <alphamap_pars_fragment>
#include <alphatest_pars_fragment>
#include <alphahash_pars_fragment>
#include <fog_pars_fragment>
#include <normal_pars_fragment>
#include <bumpmap_pars_fragment>
#include <normalmap_pars_fragment>
#include <logdepthbuf_pars_fragment>
#include <clipping_planes_pars_fragment>
void main() {
	vec4 diffuseColor = vec4( diffuse, opacity );
	#include <clipping_planes_fragment>
	#include <logdepthbuf_fragment>
	#include <map_fragment>
	#include <color_fragment>
	#include <alphamap_fragment>
	#include <alphatest_fragment>
	#include <alphahash_fragment>
	#include <normal_fragment_begin>
	#include <normal_fragment_maps>
	vec3 viewDir = normalize( vViewPosition );
	vec3 x = normalize( vec3( viewDir.z, 0.0, - viewDir.x ) );
	vec3 y = cross( viewDir, x );
	vec2 uv = vec2( dot( x, normal ), dot( y, normal ) ) * 0.495 + 0.5;
	#ifdef USE_MATCAP
		vec4 matcapColor = texture2D( matcap, uv );
	#else
		vec4 matcapColor = vec4( vec3( mix( 0.2, 0.8, uv.y ) ), 1.0 );
	#endif
	vec3 outgoingLight = diffuseColor.rgb * matcapColor.rgb;
	#include <opaque_fragment>
	#include <tonemapping_fragment>
	#include <colorspace_fragment>
	#include <fog_fragment>
	#include <premultiplied_alpha_fragment>
	#include <dithering_fragment>
}`,dE=`#define NORMAL
#if defined( FLAT_SHADED ) || defined( USE_BUMPMAP ) || defined( USE_NORMALMAP_TANGENTSPACE )
	varying vec3 vViewPosition;
#endif
#include <common>
#include <batching_pars_vertex>
#include <uv_pars_vertex>
#include <displacementmap_pars_vertex>
#include <normal_pars_vertex>
#include <morphtarget_pars_vertex>
#include <skinning_pars_vertex>
#include <logdepthbuf_pars_vertex>
#include <clipping_planes_pars_vertex>
void main() {
	#include <uv_vertex>
	#include <batching_vertex>
	#include <beginnormal_vertex>
	#include <morphinstance_vertex>
	#include <morphnormal_vertex>
	#include <skinbase_vertex>
	#include <skinnormal_vertex>
	#include <defaultnormal_vertex>
	#include <normal_vertex>
	#include <begin_vertex>
	#include <morphtarget_vertex>
	#include <skinning_vertex>
	#include <displacementmap_vertex>
	#include <project_vertex>
	#include <logdepthbuf_vertex>
	#include <clipping_planes_vertex>
#if defined( FLAT_SHADED ) || defined( USE_BUMPMAP ) || defined( USE_NORMALMAP_TANGENTSPACE )
	vViewPosition = - mvPosition.xyz;
#endif
}`,fE=`#define NORMAL
uniform float opacity;
#if defined( FLAT_SHADED ) || defined( USE_BUMPMAP ) || defined( USE_NORMALMAP_TANGENTSPACE )
	varying vec3 vViewPosition;
#endif
#include <packing>
#include <uv_pars_fragment>
#include <normal_pars_fragment>
#include <bumpmap_pars_fragment>
#include <normalmap_pars_fragment>
#include <logdepthbuf_pars_fragment>
#include <clipping_planes_pars_fragment>
void main() {
	vec4 diffuseColor = vec4( 0.0, 0.0, 0.0, opacity );
	#include <clipping_planes_fragment>
	#include <logdepthbuf_fragment>
	#include <normal_fragment_begin>
	#include <normal_fragment_maps>
	gl_FragColor = vec4( packNormalToRGB( normal ), diffuseColor.a );
	#ifdef OPAQUE
		gl_FragColor.a = 1.0;
	#endif
}`,pE=`#define PHONG
varying vec3 vViewPosition;
#include <common>
#include <batching_pars_vertex>
#include <uv_pars_vertex>
#include <displacementmap_pars_vertex>
#include <envmap_pars_vertex>
#include <color_pars_vertex>
#include <fog_pars_vertex>
#include <normal_pars_vertex>
#include <morphtarget_pars_vertex>
#include <skinning_pars_vertex>
#include <shadowmap_pars_vertex>
#include <logdepthbuf_pars_vertex>
#include <clipping_planes_pars_vertex>
void main() {
	#include <uv_vertex>
	#include <color_vertex>
	#include <morphcolor_vertex>
	#include <batching_vertex>
	#include <beginnormal_vertex>
	#include <morphinstance_vertex>
	#include <morphnormal_vertex>
	#include <skinbase_vertex>
	#include <skinnormal_vertex>
	#include <defaultnormal_vertex>
	#include <normal_vertex>
	#include <begin_vertex>
	#include <morphtarget_vertex>
	#include <skinning_vertex>
	#include <displacementmap_vertex>
	#include <project_vertex>
	#include <logdepthbuf_vertex>
	#include <clipping_planes_vertex>
	vViewPosition = - mvPosition.xyz;
	#include <worldpos_vertex>
	#include <envmap_vertex>
	#include <shadowmap_vertex>
	#include <fog_vertex>
}`,mE=`#define PHONG
uniform vec3 diffuse;
uniform vec3 emissive;
uniform vec3 specular;
uniform float shininess;
uniform float opacity;
#include <common>
#include <packing>
#include <dithering_pars_fragment>
#include <color_pars_fragment>
#include <uv_pars_fragment>
#include <map_pars_fragment>
#include <alphamap_pars_fragment>
#include <alphatest_pars_fragment>
#include <alphahash_pars_fragment>
#include <aomap_pars_fragment>
#include <lightmap_pars_fragment>
#include <emissivemap_pars_fragment>
#include <envmap_common_pars_fragment>
#include <envmap_pars_fragment>
#include <fog_pars_fragment>
#include <bsdfs>
#include <lights_pars_begin>
#include <normal_pars_fragment>
#include <lights_phong_pars_fragment>
#include <shadowmap_pars_fragment>
#include <bumpmap_pars_fragment>
#include <normalmap_pars_fragment>
#include <specularmap_pars_fragment>
#include <logdepthbuf_pars_fragment>
#include <clipping_planes_pars_fragment>
void main() {
	vec4 diffuseColor = vec4( diffuse, opacity );
	#include <clipping_planes_fragment>
	ReflectedLight reflectedLight = ReflectedLight( vec3( 0.0 ), vec3( 0.0 ), vec3( 0.0 ), vec3( 0.0 ) );
	vec3 totalEmissiveRadiance = emissive;
	#include <logdepthbuf_fragment>
	#include <map_fragment>
	#include <color_fragment>
	#include <alphamap_fragment>
	#include <alphatest_fragment>
	#include <alphahash_fragment>
	#include <specularmap_fragment>
	#include <normal_fragment_begin>
	#include <normal_fragment_maps>
	#include <emissivemap_fragment>
	#include <lights_phong_fragment>
	#include <lights_fragment_begin>
	#include <lights_fragment_maps>
	#include <lights_fragment_end>
	#include <aomap_fragment>
	vec3 outgoingLight = reflectedLight.directDiffuse + reflectedLight.indirectDiffuse + reflectedLight.directSpecular + reflectedLight.indirectSpecular + totalEmissiveRadiance;
	#include <envmap_fragment>
	#include <opaque_fragment>
	#include <tonemapping_fragment>
	#include <colorspace_fragment>
	#include <fog_fragment>
	#include <premultiplied_alpha_fragment>
	#include <dithering_fragment>
}`,gE=`#define STANDARD
varying vec3 vViewPosition;
#ifdef USE_TRANSMISSION
	varying vec3 vWorldPosition;
#endif
#include <common>
#include <batching_pars_vertex>
#include <uv_pars_vertex>
#include <displacementmap_pars_vertex>
#include <color_pars_vertex>
#include <fog_pars_vertex>
#include <normal_pars_vertex>
#include <morphtarget_pars_vertex>
#include <skinning_pars_vertex>
#include <shadowmap_pars_vertex>
#include <logdepthbuf_pars_vertex>
#include <clipping_planes_pars_vertex>
void main() {
	#include <uv_vertex>
	#include <color_vertex>
	#include <morphinstance_vertex>
	#include <morphcolor_vertex>
	#include <batching_vertex>
	#include <beginnormal_vertex>
	#include <morphnormal_vertex>
	#include <skinbase_vertex>
	#include <skinnormal_vertex>
	#include <defaultnormal_vertex>
	#include <normal_vertex>
	#include <begin_vertex>
	#include <morphtarget_vertex>
	#include <skinning_vertex>
	#include <displacementmap_vertex>
	#include <project_vertex>
	#include <logdepthbuf_vertex>
	#include <clipping_planes_vertex>
	vViewPosition = - mvPosition.xyz;
	#include <worldpos_vertex>
	#include <shadowmap_vertex>
	#include <fog_vertex>
#ifdef USE_TRANSMISSION
	vWorldPosition = worldPosition.xyz;
#endif
}`,_E=`#define STANDARD
#ifdef PHYSICAL
	#define IOR
	#define USE_SPECULAR
#endif
uniform vec3 diffuse;
uniform vec3 emissive;
uniform float roughness;
uniform float metalness;
uniform float opacity;
#ifdef IOR
	uniform float ior;
#endif
#ifdef USE_SPECULAR
	uniform float specularIntensity;
	uniform vec3 specularColor;
	#ifdef USE_SPECULAR_COLORMAP
		uniform sampler2D specularColorMap;
	#endif
	#ifdef USE_SPECULAR_INTENSITYMAP
		uniform sampler2D specularIntensityMap;
	#endif
#endif
#ifdef USE_CLEARCOAT
	uniform float clearcoat;
	uniform float clearcoatRoughness;
#endif
#ifdef USE_DISPERSION
	uniform float dispersion;
#endif
#ifdef USE_IRIDESCENCE
	uniform float iridescence;
	uniform float iridescenceIOR;
	uniform float iridescenceThicknessMinimum;
	uniform float iridescenceThicknessMaximum;
#endif
#ifdef USE_SHEEN
	uniform vec3 sheenColor;
	uniform float sheenRoughness;
	#ifdef USE_SHEEN_COLORMAP
		uniform sampler2D sheenColorMap;
	#endif
	#ifdef USE_SHEEN_ROUGHNESSMAP
		uniform sampler2D sheenRoughnessMap;
	#endif
#endif
#ifdef USE_ANISOTROPY
	uniform vec2 anisotropyVector;
	#ifdef USE_ANISOTROPYMAP
		uniform sampler2D anisotropyMap;
	#endif
#endif
varying vec3 vViewPosition;
#include <common>
#include <packing>
#include <dithering_pars_fragment>
#include <color_pars_fragment>
#include <uv_pars_fragment>
#include <map_pars_fragment>
#include <alphamap_pars_fragment>
#include <alphatest_pars_fragment>
#include <alphahash_pars_fragment>
#include <aomap_pars_fragment>
#include <lightmap_pars_fragment>
#include <emissivemap_pars_fragment>
#include <iridescence_fragment>
#include <cube_uv_reflection_fragment>
#include <envmap_common_pars_fragment>
#include <envmap_physical_pars_fragment>
#include <fog_pars_fragment>
#include <lights_pars_begin>
#include <normal_pars_fragment>
#include <lights_physical_pars_fragment>
#include <transmission_pars_fragment>
#include <shadowmap_pars_fragment>
#include <bumpmap_pars_fragment>
#include <normalmap_pars_fragment>
#include <clearcoat_pars_fragment>
#include <iridescence_pars_fragment>
#include <roughnessmap_pars_fragment>
#include <metalnessmap_pars_fragment>
#include <logdepthbuf_pars_fragment>
#include <clipping_planes_pars_fragment>
void main() {
	vec4 diffuseColor = vec4( diffuse, opacity );
	#include <clipping_planes_fragment>
	ReflectedLight reflectedLight = ReflectedLight( vec3( 0.0 ), vec3( 0.0 ), vec3( 0.0 ), vec3( 0.0 ) );
	vec3 totalEmissiveRadiance = emissive;
	#include <logdepthbuf_fragment>
	#include <map_fragment>
	#include <color_fragment>
	#include <alphamap_fragment>
	#include <alphatest_fragment>
	#include <alphahash_fragment>
	#include <roughnessmap_fragment>
	#include <metalnessmap_fragment>
	#include <normal_fragment_begin>
	#include <normal_fragment_maps>
	#include <clearcoat_normal_fragment_begin>
	#include <clearcoat_normal_fragment_maps>
	#include <emissivemap_fragment>
	#include <lights_physical_fragment>
	#include <lights_fragment_begin>
	#include <lights_fragment_maps>
	#include <lights_fragment_end>
	#include <aomap_fragment>
	vec3 totalDiffuse = reflectedLight.directDiffuse + reflectedLight.indirectDiffuse;
	vec3 totalSpecular = reflectedLight.directSpecular + reflectedLight.indirectSpecular;
	#include <transmission_fragment>
	vec3 outgoingLight = totalDiffuse + totalSpecular + totalEmissiveRadiance;
	#ifdef USE_SHEEN
		float sheenEnergyComp = 1.0 - 0.157 * max3( material.sheenColor );
		outgoingLight = outgoingLight * sheenEnergyComp + sheenSpecularDirect + sheenSpecularIndirect;
	#endif
	#ifdef USE_CLEARCOAT
		float dotNVcc = saturate( dot( geometryClearcoatNormal, geometryViewDir ) );
		vec3 Fcc = F_Schlick( material.clearcoatF0, material.clearcoatF90, dotNVcc );
		outgoingLight = outgoingLight * ( 1.0 - material.clearcoat * Fcc ) + ( clearcoatSpecularDirect + clearcoatSpecularIndirect ) * material.clearcoat;
	#endif
	#include <opaque_fragment>
	#include <tonemapping_fragment>
	#include <colorspace_fragment>
	#include <fog_fragment>
	#include <premultiplied_alpha_fragment>
	#include <dithering_fragment>
}`,vE=`#define TOON
varying vec3 vViewPosition;
#include <common>
#include <batching_pars_vertex>
#include <uv_pars_vertex>
#include <displacementmap_pars_vertex>
#include <color_pars_vertex>
#include <fog_pars_vertex>
#include <normal_pars_vertex>
#include <morphtarget_pars_vertex>
#include <skinning_pars_vertex>
#include <shadowmap_pars_vertex>
#include <logdepthbuf_pars_vertex>
#include <clipping_planes_pars_vertex>
void main() {
	#include <uv_vertex>
	#include <color_vertex>
	#include <morphinstance_vertex>
	#include <morphcolor_vertex>
	#include <batching_vertex>
	#include <beginnormal_vertex>
	#include <morphnormal_vertex>
	#include <skinbase_vertex>
	#include <skinnormal_vertex>
	#include <defaultnormal_vertex>
	#include <normal_vertex>
	#include <begin_vertex>
	#include <morphtarget_vertex>
	#include <skinning_vertex>
	#include <displacementmap_vertex>
	#include <project_vertex>
	#include <logdepthbuf_vertex>
	#include <clipping_planes_vertex>
	vViewPosition = - mvPosition.xyz;
	#include <worldpos_vertex>
	#include <shadowmap_vertex>
	#include <fog_vertex>
}`,yE=`#define TOON
uniform vec3 diffuse;
uniform vec3 emissive;
uniform float opacity;
#include <common>
#include <packing>
#include <dithering_pars_fragment>
#include <color_pars_fragment>
#include <uv_pars_fragment>
#include <map_pars_fragment>
#include <alphamap_pars_fragment>
#include <alphatest_pars_fragment>
#include <alphahash_pars_fragment>
#include <aomap_pars_fragment>
#include <lightmap_pars_fragment>
#include <emissivemap_pars_fragment>
#include <gradientmap_pars_fragment>
#include <fog_pars_fragment>
#include <bsdfs>
#include <lights_pars_begin>
#include <normal_pars_fragment>
#include <lights_toon_pars_fragment>
#include <shadowmap_pars_fragment>
#include <bumpmap_pars_fragment>
#include <normalmap_pars_fragment>
#include <logdepthbuf_pars_fragment>
#include <clipping_planes_pars_fragment>
void main() {
	vec4 diffuseColor = vec4( diffuse, opacity );
	#include <clipping_planes_fragment>
	ReflectedLight reflectedLight = ReflectedLight( vec3( 0.0 ), vec3( 0.0 ), vec3( 0.0 ), vec3( 0.0 ) );
	vec3 totalEmissiveRadiance = emissive;
	#include <logdepthbuf_fragment>
	#include <map_fragment>
	#include <color_fragment>
	#include <alphamap_fragment>
	#include <alphatest_fragment>
	#include <alphahash_fragment>
	#include <normal_fragment_begin>
	#include <normal_fragment_maps>
	#include <emissivemap_fragment>
	#include <lights_toon_fragment>
	#include <lights_fragment_begin>
	#include <lights_fragment_maps>
	#include <lights_fragment_end>
	#include <aomap_fragment>
	vec3 outgoingLight = reflectedLight.directDiffuse + reflectedLight.indirectDiffuse + totalEmissiveRadiance;
	#include <opaque_fragment>
	#include <tonemapping_fragment>
	#include <colorspace_fragment>
	#include <fog_fragment>
	#include <premultiplied_alpha_fragment>
	#include <dithering_fragment>
}`,xE=`uniform float size;
uniform float scale;
#include <common>
#include <color_pars_vertex>
#include <fog_pars_vertex>
#include <morphtarget_pars_vertex>
#include <logdepthbuf_pars_vertex>
#include <clipping_planes_pars_vertex>
#ifdef USE_POINTS_UV
	varying vec2 vUv;
	uniform mat3 uvTransform;
#endif
void main() {
	#ifdef USE_POINTS_UV
		vUv = ( uvTransform * vec3( uv, 1 ) ).xy;
	#endif
	#include <color_vertex>
	#include <morphinstance_vertex>
	#include <morphcolor_vertex>
	#include <begin_vertex>
	#include <morphtarget_vertex>
	#include <project_vertex>
	gl_PointSize = size;
	#ifdef USE_SIZEATTENUATION
		bool isPerspective = isPerspectiveMatrix( projectionMatrix );
		if ( isPerspective ) gl_PointSize *= ( scale / - mvPosition.z );
	#endif
	#include <logdepthbuf_vertex>
	#include <clipping_planes_vertex>
	#include <worldpos_vertex>
	#include <fog_vertex>
}`,wE=`uniform vec3 diffuse;
uniform float opacity;
#include <common>
#include <color_pars_fragment>
#include <map_particle_pars_fragment>
#include <alphatest_pars_fragment>
#include <alphahash_pars_fragment>
#include <fog_pars_fragment>
#include <logdepthbuf_pars_fragment>
#include <clipping_planes_pars_fragment>
void main() {
	vec4 diffuseColor = vec4( diffuse, opacity );
	#include <clipping_planes_fragment>
	vec3 outgoingLight = vec3( 0.0 );
	#include <logdepthbuf_fragment>
	#include <map_particle_fragment>
	#include <color_fragment>
	#include <alphatest_fragment>
	#include <alphahash_fragment>
	outgoingLight = diffuseColor.rgb;
	#include <opaque_fragment>
	#include <tonemapping_fragment>
	#include <colorspace_fragment>
	#include <fog_fragment>
	#include <premultiplied_alpha_fragment>
}`,SE=`#include <common>
#include <batching_pars_vertex>
#include <fog_pars_vertex>
#include <morphtarget_pars_vertex>
#include <skinning_pars_vertex>
#include <logdepthbuf_pars_vertex>
#include <shadowmap_pars_vertex>
void main() {
	#include <batching_vertex>
	#include <beginnormal_vertex>
	#include <morphinstance_vertex>
	#include <morphnormal_vertex>
	#include <skinbase_vertex>
	#include <skinnormal_vertex>
	#include <defaultnormal_vertex>
	#include <begin_vertex>
	#include <morphtarget_vertex>
	#include <skinning_vertex>
	#include <project_vertex>
	#include <logdepthbuf_vertex>
	#include <worldpos_vertex>
	#include <shadowmap_vertex>
	#include <fog_vertex>
}`,ME=`uniform vec3 color;
uniform float opacity;
#include <common>
#include <packing>
#include <fog_pars_fragment>
#include <bsdfs>
#include <lights_pars_begin>
#include <logdepthbuf_pars_fragment>
#include <shadowmap_pars_fragment>
#include <shadowmask_pars_fragment>
void main() {
	#include <logdepthbuf_fragment>
	gl_FragColor = vec4( color, opacity * ( 1.0 - getShadowMask() ) );
	#include <tonemapping_fragment>
	#include <colorspace_fragment>
	#include <fog_fragment>
}`,EE=`uniform float rotation;
uniform vec2 center;
#include <common>
#include <uv_pars_vertex>
#include <fog_pars_vertex>
#include <logdepthbuf_pars_vertex>
#include <clipping_planes_pars_vertex>
void main() {
	#include <uv_vertex>
	vec4 mvPosition = modelViewMatrix * vec4( 0.0, 0.0, 0.0, 1.0 );
	vec2 scale;
	scale.x = length( vec3( modelMatrix[ 0 ].x, modelMatrix[ 0 ].y, modelMatrix[ 0 ].z ) );
	scale.y = length( vec3( modelMatrix[ 1 ].x, modelMatrix[ 1 ].y, modelMatrix[ 1 ].z ) );
	#ifndef USE_SIZEATTENUATION
		bool isPerspective = isPerspectiveMatrix( projectionMatrix );
		if ( isPerspective ) scale *= - mvPosition.z;
	#endif
	vec2 alignedPosition = ( position.xy - ( center - vec2( 0.5 ) ) ) * scale;
	vec2 rotatedPosition;
	rotatedPosition.x = cos( rotation ) * alignedPosition.x - sin( rotation ) * alignedPosition.y;
	rotatedPosition.y = sin( rotation ) * alignedPosition.x + cos( rotation ) * alignedPosition.y;
	mvPosition.xy += rotatedPosition;
	gl_Position = projectionMatrix * mvPosition;
	#include <logdepthbuf_vertex>
	#include <clipping_planes_vertex>
	#include <fog_vertex>
}`,CE=`uniform vec3 diffuse;
uniform float opacity;
#include <common>
#include <uv_pars_fragment>
#include <map_pars_fragment>
#include <alphamap_pars_fragment>
#include <alphatest_pars_fragment>
#include <alphahash_pars_fragment>
#include <fog_pars_fragment>
#include <logdepthbuf_pars_fragment>
#include <clipping_planes_pars_fragment>
void main() {
	vec4 diffuseColor = vec4( diffuse, opacity );
	#include <clipping_planes_fragment>
	vec3 outgoingLight = vec3( 0.0 );
	#include <logdepthbuf_fragment>
	#include <map_fragment>
	#include <alphamap_fragment>
	#include <alphatest_fragment>
	#include <alphahash_fragment>
	outgoingLight = diffuseColor.rgb;
	#include <opaque_fragment>
	#include <tonemapping_fragment>
	#include <colorspace_fragment>
	#include <fog_fragment>
}`,Le={alphahash_fragment:qS,alphahash_pars_fragment:$S,alphamap_fragment:ZS,alphamap_pars_fragment:JS,alphatest_fragment:KS,alphatest_pars_fragment:jS,aomap_fragment:QS,aomap_pars_fragment:e1,batching_pars_vertex:t1,batching_vertex:n1,begin_vertex:i1,beginnormal_vertex:r1,bsdfs:s1,iridescence_fragment:o1,bumpmap_pars_fragment:a1,clipping_planes_fragment:l1,clipping_planes_pars_fragment:c1,clipping_planes_pars_vertex:u1,clipping_planes_vertex:h1,color_fragment:d1,color_pars_fragment:f1,color_pars_vertex:p1,color_vertex:m1,common:g1,cube_uv_reflection_fragment:_1,defaultnormal_vertex:v1,displacementmap_pars_vertex:y1,displacementmap_vertex:x1,emissivemap_fragment:w1,emissivemap_pars_fragment:S1,colorspace_fragment:M1,colorspace_pars_fragment:E1,envmap_fragment:C1,envmap_common_pars_fragment:T1,envmap_pars_fragment:b1,envmap_pars_vertex:A1,envmap_physical_pars_fragment:k1,envmap_vertex:R1,fog_vertex:P1,fog_pars_vertex:I1,fog_fragment:L1,fog_pars_fragment:U1,gradientmap_pars_fragment:N1,lightmap_pars_fragment:D1,lights_lambert_fragment:O1,lights_lambert_pars_fragment:F1,lights_pars_begin:B1,lights_toon_fragment:z1,lights_toon_pars_fragment:V1,lights_phong_fragment:H1,lights_phong_pars_fragment:G1,lights_physical_fragment:W1,lights_physical_pars_fragment:X1,lights_fragment_begin:Y1,lights_fragment_maps:q1,lights_fragment_end:$1,logdepthbuf_fragment:Z1,logdepthbuf_pars_fragment:J1,logdepthbuf_pars_vertex:K1,logdepthbuf_vertex:j1,map_fragment:Q1,map_pars_fragment:eM,map_particle_fragment:tM,map_particle_pars_fragment:nM,metalnessmap_fragment:iM,metalnessmap_pars_fragment:rM,morphinstance_vertex:sM,morphcolor_vertex:oM,morphnormal_vertex:aM,morphtarget_pars_vertex:lM,morphtarget_vertex:cM,normal_fragment_begin:uM,normal_fragment_maps:hM,normal_pars_fragment:dM,normal_pars_vertex:fM,normal_vertex:pM,normalmap_pars_fragment:mM,clearcoat_normal_fragment_begin:gM,clearcoat_normal_fragment_maps:_M,clearcoat_pars_fragment:vM,iridescence_pars_fragment:yM,opaque_fragment:xM,packing:wM,premultiplied_alpha_fragment:SM,project_vertex:MM,dithering_fragment:EM,dithering_pars_fragment:CM,roughnessmap_fragment:TM,roughnessmap_pars_fragment:bM,shadowmap_pars_fragment:AM,shadowmap_pars_vertex:RM,shadowmap_vertex:PM,shadowmask_pars_fragment:IM,skinbase_vertex:LM,skinning_pars_vertex:UM,skinning_vertex:NM,skinnormal_vertex:DM,specularmap_fragment:OM,specularmap_pars_fragment:FM,tonemapping_fragment:BM,tonemapping_pars_fragment:kM,transmission_fragment:zM,transmission_pars_fragment:VM,uv_pars_fragment:HM,uv_pars_vertex:GM,uv_vertex:WM,worldpos_vertex:XM,background_vert:YM,background_frag:qM,backgroundCube_vert:$M,backgroundCube_frag:ZM,cube_vert:JM,cube_frag:KM,depth_vert:jM,depth_frag:QM,distanceRGBA_vert:eE,distanceRGBA_frag:tE,equirect_vert:nE,equirect_frag:iE,linedashed_vert:rE,linedashed_frag:sE,meshbasic_vert:oE,meshbasic_frag:aE,meshlambert_vert:lE,meshlambert_frag:cE,meshmatcap_vert:uE,meshmatcap_frag:hE,meshnormal_vert:dE,meshnormal_frag:fE,meshphong_vert:pE,meshphong_frag:mE,meshphysical_vert:gE,meshphysical_frag:_E,meshtoon_vert:vE,meshtoon_frag:yE,points_vert:xE,points_frag:wE,shadow_vert:SE,shadow_frag:ME,sprite_vert:EE,sprite_frag:CE},re={common:{diffuse:{value:new Oe(16777215)},opacity:{value:1},map:{value:null},mapTransform:{value:new Ue},alphaMap:{value:null},alphaMapTransform:{value:new Ue},alphaTest:{value:0}},specularmap:{specularMap:{value:null},specularMapTransform:{value:new Ue}},envmap:{envMap:{value:null},envMapRotation:{value:new Ue},flipEnvMap:{value:-1},reflectivity:{value:1},ior:{value:1.5},refractionRatio:{value:.98}},aomap:{aoMap:{value:null},aoMapIntensity:{value:1},aoMapTransform:{value:new Ue}},lightmap:{lightMap:{value:null},lightMapIntensity:{value:1},lightMapTransform:{value:new Ue}},bumpmap:{bumpMap:{value:null},bumpMapTransform:{value:new Ue},bumpScale:{value:1}},normalmap:{normalMap:{value:null},normalMapTransform:{value:new Ue},normalScale:{value:new ze(1,1)}},displacementmap:{displacementMap:{value:null},displacementMapTransform:{value:new Ue},displacementScale:{value:1},displacementBias:{value:0}},emissivemap:{emissiveMap:{value:null},emissiveMapTransform:{value:new Ue}},metalnessmap:{metalnessMap:{value:null},metalnessMapTransform:{value:new Ue}},roughnessmap:{roughnessMap:{value:null},roughnessMapTransform:{value:new Ue}},gradientmap:{gradientMap:{value:null}},fog:{fogDensity:{value:25e-5},fogNear:{value:1},fogFar:{value:2e3},fogColor:{value:new Oe(16777215)}},lights:{ambientLightColor:{value:[]},lightProbe:{value:[]},directionalLights:{value:[],properties:{direction:{},color:{}}},directionalLightShadows:{value:[],properties:{shadowIntensity:1,shadowBias:{},shadowNormalBias:{},shadowRadius:{},shadowMapSize:{}}},directionalShadowMap:{value:[]},directionalShadowMatrix:{value:[]},spotLights:{value:[],properties:{color:{},position:{},direction:{},distance:{},coneCos:{},penumbraCos:{},decay:{}}},spotLightShadows:{value:[],properties:{shadowIntensity:1,shadowBias:{},shadowNormalBias:{},shadowRadius:{},shadowMapSize:{}}},spotLightMap:{value:[]},spotShadowMap:{value:[]},spotLightMatrix:{value:[]},pointLights:{value:[],properties:{color:{},position:{},decay:{},distance:{}}},pointLightShadows:{value:[],properties:{shadowIntensity:1,shadowBias:{},shadowNormalBias:{},shadowRadius:{},shadowMapSize:{},shadowCameraNear:{},shadowCameraFar:{}}},pointShadowMap:{value:[]},pointShadowMatrix:{value:[]},hemisphereLights:{value:[],properties:{direction:{},skyColor:{},groundColor:{}}},rectAreaLights:{value:[],properties:{color:{},position:{},width:{},height:{}}},ltc_1:{value:null},ltc_2:{value:null}},points:{diffuse:{value:new Oe(16777215)},opacity:{value:1},size:{value:1},scale:{value:1},map:{value:null},alphaMap:{value:null},alphaMapTransform:{value:new Ue},alphaTest:{value:0},uvTransform:{value:new Ue}},sprite:{diffuse:{value:new Oe(16777215)},opacity:{value:1},center:{value:new ze(.5,.5)},rotation:{value:0},map:{value:null},mapTransform:{value:new Ue},alphaMap:{value:null},alphaMapTransform:{value:new Ue},alphaTest:{value:0}}},Wn={basic:{uniforms:Xt([re.common,re.specularmap,re.envmap,re.aomap,re.lightmap,re.fog]),vertexShader:Le.meshbasic_vert,fragmentShader:Le.meshbasic_frag},lambert:{uniforms:Xt([re.common,re.specularmap,re.envmap,re.aomap,re.lightmap,re.emissivemap,re.bumpmap,re.normalmap,re.displacementmap,re.fog,re.lights,{emissive:{value:new Oe(0)}}]),vertexShader:Le.meshlambert_vert,fragmentShader:Le.meshlambert_frag},phong:{uniforms:Xt([re.common,re.specularmap,re.envmap,re.aomap,re.lightmap,re.emissivemap,re.bumpmap,re.normalmap,re.displacementmap,re.fog,re.lights,{emissive:{value:new Oe(0)},specular:{value:new Oe(1118481)},shininess:{value:30}}]),vertexShader:Le.meshphong_vert,fragmentShader:Le.meshphong_frag},standard:{uniforms:Xt([re.common,re.envmap,re.aomap,re.lightmap,re.emissivemap,re.bumpmap,re.normalmap,re.displacementmap,re.roughnessmap,re.metalnessmap,re.fog,re.lights,{emissive:{value:new Oe(0)},roughness:{value:1},metalness:{value:0},envMapIntensity:{value:1}}]),vertexShader:Le.meshphysical_vert,fragmentShader:Le.meshphysical_frag},toon:{uniforms:Xt([re.common,re.aomap,re.lightmap,re.emissivemap,re.bumpmap,re.normalmap,re.displacementmap,re.gradientmap,re.fog,re.lights,{emissive:{value:new Oe(0)}}]),vertexShader:Le.meshtoon_vert,fragmentShader:Le.meshtoon_frag},matcap:{uniforms:Xt([re.common,re.bumpmap,re.normalmap,re.displacementmap,re.fog,{matcap:{value:null}}]),vertexShader:Le.meshmatcap_vert,fragmentShader:Le.meshmatcap_frag},points:{uniforms:Xt([re.points,re.fog]),vertexShader:Le.points_vert,fragmentShader:Le.points_frag},dashed:{uniforms:Xt([re.common,re.fog,{scale:{value:1},dashSize:{value:1},totalSize:{value:2}}]),vertexShader:Le.linedashed_vert,fragmentShader:Le.linedashed_frag},depth:{uniforms:Xt([re.common,re.displacementmap]),vertexShader:Le.depth_vert,fragmentShader:Le.depth_frag},normal:{uniforms:Xt([re.common,re.bumpmap,re.normalmap,re.displacementmap,{opacity:{value:1}}]),vertexShader:Le.meshnormal_vert,fragmentShader:Le.meshnormal_frag},sprite:{uniforms:Xt([re.sprite,re.fog]),vertexShader:Le.sprite_vert,fragmentShader:Le.sprite_frag},background:{uniforms:{uvTransform:{value:new Ue},t2D:{value:null},backgroundIntensity:{value:1}},vertexShader:Le.background_vert,fragmentShader:Le.background_frag},backgroundCube:{uniforms:{envMap:{value:null},flipEnvMap:{value:-1},backgroundBlurriness:{value:0},backgroundIntensity:{value:1},backgroundRotation:{value:new Ue}},vertexShader:Le.backgroundCube_vert,fragmentShader:Le.backgroundCube_frag},cube:{uniforms:{tCube:{value:null},tFlip:{value:-1},opacity:{value:1}},vertexShader:Le.cube_vert,fragmentShader:Le.cube_frag},equirect:{uniforms:{tEquirect:{value:null}},vertexShader:Le.equirect_vert,fragmentShader:Le.equirect_frag},distanceRGBA:{uniforms:Xt([re.common,re.displacementmap,{referencePosition:{value:new D},nearDistance:{value:1},farDistance:{value:1e3}}]),vertexShader:Le.distanceRGBA_vert,fragmentShader:Le.distanceRGBA_frag},shadow:{uniforms:Xt([re.lights,re.fog,{color:{value:new Oe(0)},opacity:{value:1}}]),vertexShader:Le.shadow_vert,fragmentShader:Le.shadow_frag}};Wn.physical={uniforms:Xt([Wn.standard.uniforms,{clearcoat:{value:0},clearcoatMap:{value:null},clearcoatMapTransform:{value:new Ue},clearcoatNormalMap:{value:null},clearcoatNormalMapTransform:{value:new Ue},clearcoatNormalScale:{value:new ze(1,1)},clearcoatRoughness:{value:0},clearcoatRoughnessMap:{value:null},clearcoatRoughnessMapTransform:{value:new Ue},dispersion:{value:0},iridescence:{value:0},iridescenceMap:{value:null},iridescenceMapTransform:{value:new Ue},iridescenceIOR:{value:1.3},iridescenceThicknessMinimum:{value:100},iridescenceThicknessMaximum:{value:400},iridescenceThicknessMap:{value:null},iridescenceThicknessMapTransform:{value:new Ue},sheen:{value:0},sheenColor:{value:new Oe(0)},sheenColorMap:{value:null},sheenColorMapTransform:{value:new Ue},sheenRoughness:{value:1},sheenRoughnessMap:{value:null},sheenRoughnessMapTransform:{value:new Ue},transmission:{value:0},transmissionMap:{value:null},transmissionMapTransform:{value:new Ue},transmissionSamplerSize:{value:new ze},transmissionSamplerMap:{value:null},thickness:{value:0},thicknessMap:{value:null},thicknessMapTransform:{value:new Ue},attenuationDistance:{value:0},attenuationColor:{value:new Oe(0)},specularColor:{value:new Oe(1,1,1)},specularColorMap:{value:null},specularColorMapTransform:{value:new Ue},specularIntensity:{value:1},specularIntensityMap:{value:null},specularIntensityMapTransform:{value:new Ue},anisotropyVector:{value:new ze},anisotropyMap:{value:null},anisotropyMapTransform:{value:new Ue}}]),vertexShader:Le.meshphysical_vert,fragmentShader:Le.meshphysical_frag};var jl={r:0,b:0,g:0},yr=new Xn,TE=new et;function bE(n,e,t,i,r,s,o){let a=new Oe(0),l=s===!0?0:1,c,u,p=null,f=0,m=null;function _(g){let v=g.isScene===!0?g.background:null;return v&&v.isTexture&&(v=(g.backgroundBlurriness>0?t:e).get(v)),v}function y(g){let v=!1,x=_(g);x===null?h(a,l):x&&x.isColor&&(h(x,1),v=!0);let R=n.xr.getEnvironmentBlendMode();R==="additive"?i.buffers.color.setClear(0,0,0,1,o):R==="alpha-blend"&&i.buffers.color.setClear(0,0,0,0,o),(n.autoClear||v)&&(i.buffers.depth.setTest(!0),i.buffers.depth.setMask(!0),i.buffers.color.setMask(!0),n.clear(n.autoClearColor,n.autoClearDepth,n.autoClearStencil))}function d(g,v){let x=_(v);x&&(x.isCubeTexture||x.mapping===Lc)?(u===void 0&&(u=new St(new Pr(1,1,1),new Mn({name:"BackgroundCubeMaterial",uniforms:Vs(Wn.backgroundCube.uniforms),vertexShader:Wn.backgroundCube.vertexShader,fragmentShader:Wn.backgroundCube.fragmentShader,side:tn,depthTest:!1,depthWrite:!1,fog:!1})),u.geometry.deleteAttribute("normal"),u.geometry.deleteAttribute("uv"),u.onBeforeRender=function(R,C,T){this.matrixWorld.copyPosition(T.matrixWorld)},Object.defineProperty(u.material,"envMap",{get:function(){return this.uniforms.envMap.value}}),r.update(u)),yr.copy(v.backgroundRotation),yr.x*=-1,yr.y*=-1,yr.z*=-1,x.isCubeTexture&&x.isRenderTargetTexture===!1&&(yr.y*=-1,yr.z*=-1),u.material.uniforms.envMap.value=x,u.material.uniforms.flipEnvMap.value=x.isCubeTexture&&x.isRenderTargetTexture===!1?-1:1,u.material.uniforms.backgroundBlurriness.value=v.backgroundBlurriness,u.material.uniforms.backgroundIntensity.value=v.backgroundIntensity,u.material.uniforms.backgroundRotation.value.setFromMatrix4(TE.makeRotationFromEuler(yr)),u.material.toneMapped=Ke.getTransfer(x.colorSpace)!==rt,(p!==x||f!==x.version||m!==n.toneMapping)&&(u.material.needsUpdate=!0,p=x,f=x.version,m=n.toneMapping),u.layers.enableAll(),g.unshift(u,u.geometry,u.material,0,0,null)):x&&x.isTexture&&(c===void 0&&(c=new St(new Yn(2,2),new Mn({name:"BackgroundMaterial",uniforms:Vs(Wn.background.uniforms),vertexShader:Wn.background.vertexShader,fragmentShader:Wn.background.fragmentShader,side:$i,depthTest:!1,depthWrite:!1,fog:!1})),c.geometry.deleteAttribute("normal"),Object.defineProperty(c.material,"map",{get:function(){return this.uniforms.t2D.value}}),r.update(c)),c.material.uniforms.t2D.value=x,c.material.uniforms.backgroundIntensity.value=v.backgroundIntensity,c.material.toneMapped=Ke.getTransfer(x.colorSpace)!==rt,x.matrixAutoUpdate===!0&&x.updateMatrix(),c.material.uniforms.uvTransform.value.copy(x.matrix),(p!==x||f!==x.version||m!==n.toneMapping)&&(c.material.needsUpdate=!0,p=x,f=x.version,m=n.toneMapping),c.layers.enableAll(),g.unshift(c,c.geometry,c.material,0,0,null))}function h(g,v){g.getRGB(jl,Uv(n)),i.buffers.color.setClear(jl.r,jl.g,jl.b,v,o)}return{getClearColor:function(){return a},setClearColor:function(g,v=1){a.set(g),l=v,h(a,l)},getClearAlpha:function(){return l},setClearAlpha:function(g){l=g,h(a,l)},render:y,addToRenderList:d}}function AE(n,e){let t=n.getParameter(n.MAX_VERTEX_ATTRIBS),i={},r=f(null),s=r,o=!1;function a(M,P,W,k,G){let $=!1,V=p(k,W,P);s!==V&&(s=V,c(s.object)),$=m(M,k,W,G),$&&_(M,k,W,G),G!==null&&e.update(G,n.ELEMENT_ARRAY_BUFFER),($||o)&&(o=!1,x(M,P,W,k),G!==null&&n.bindBuffer(n.ELEMENT_ARRAY_BUFFER,e.get(G).buffer))}function l(){return n.createVertexArray()}function c(M){return n.bindVertexArray(M)}function u(M){return n.deleteVertexArray(M)}function p(M,P,W){let k=W.wireframe===!0,G=i[M.id];G===void 0&&(G={},i[M.id]=G);let $=G[P.id];$===void 0&&($={},G[P.id]=$);let V=$[k];return V===void 0&&(V=f(l()),$[k]=V),V}function f(M){let P=[],W=[],k=[];for(let G=0;G<t;G++)P[G]=0,W[G]=0,k[G]=0;return{geometry:null,program:null,wireframe:!1,newAttributes:P,enabledAttributes:W,attributeDivisors:k,object:M,attributes:{},index:null}}function m(M,P,W,k){let G=s.attributes,$=P.attributes,V=0,K=W.getAttributes();for(let z in K)if(K[z].location>=0){let ge=G[z],_e=$[z];if(_e===void 0&&(z==="instanceMatrix"&&M.instanceMatrix&&(_e=M.instanceMatrix),z==="instanceColor"&&M.instanceColor&&(_e=M.instanceColor)),ge===void 0||ge.attribute!==_e||_e&&ge.data!==_e.data)return!0;V++}return s.attributesNum!==V||s.index!==k}function _(M,P,W,k){let G={},$=P.attributes,V=0,K=W.getAttributes();for(let z in K)if(K[z].location>=0){let ge=$[z];ge===void 0&&(z==="instanceMatrix"&&M.instanceMatrix&&(ge=M.instanceMatrix),z==="instanceColor"&&M.instanceColor&&(ge=M.instanceColor));let _e={};_e.attribute=ge,ge&&ge.data&&(_e.data=ge.data),G[z]=_e,V++}s.attributes=G,s.attributesNum=V,s.index=k}function y(){let M=s.newAttributes;for(let P=0,W=M.length;P<W;P++)M[P]=0}function d(M){h(M,0)}function h(M,P){let W=s.newAttributes,k=s.enabledAttributes,G=s.attributeDivisors;W[M]=1,k[M]===0&&(n.enableVertexAttribArray(M),k[M]=1),G[M]!==P&&(n.vertexAttribDivisor(M,P),G[M]=P)}function g(){let M=s.newAttributes,P=s.enabledAttributes;for(let W=0,k=P.length;W<k;W++)P[W]!==M[W]&&(n.disableVertexAttribArray(W),P[W]=0)}function v(M,P,W,k,G,$,V){V===!0?n.vertexAttribIPointer(M,P,W,G,$):n.vertexAttribPointer(M,P,W,k,G,$)}function x(M,P,W,k){y();let G=k.attributes,$=W.getAttributes(),V=P.defaultAttributeValues;for(let K in $){let z=$[K];if(z.location>=0){let he=G[K];if(he===void 0&&(K==="instanceMatrix"&&M.instanceMatrix&&(he=M.instanceMatrix),K==="instanceColor"&&M.instanceColor&&(he=M.instanceColor)),he!==void 0){let ge=he.normalized,_e=he.itemSize,Ve=e.get(he);if(Ve===void 0)continue;let je=Ve.buffer,H=Ve.type,Q=Ve.bytesPerElement,fe=H===n.INT||H===n.UNSIGNED_INT||he.gpuType===pp;if(he.isInterleavedBufferAttribute){let ce=he.data,Re=ce.stride,Ne=he.offset;if(ce.isInstancedInterleavedBuffer){for(let Be=0;Be<z.locationSize;Be++)h(z.location+Be,ce.meshPerAttribute);M.isInstancedMesh!==!0&&k._maxInstanceCount===void 0&&(k._maxInstanceCount=ce.meshPerAttribute*ce.count)}else for(let Be=0;Be<z.locationSize;Be++)d(z.location+Be);n.bindBuffer(n.ARRAY_BUFFER,je);for(let Be=0;Be<z.locationSize;Be++)v(z.location+Be,_e/z.locationSize,H,ge,Re*Q,(Ne+_e/z.locationSize*Be)*Q,fe)}else{if(he.isInstancedBufferAttribute){for(let ce=0;ce<z.locationSize;ce++)h(z.location+ce,he.meshPerAttribute);M.isInstancedMesh!==!0&&k._maxInstanceCount===void 0&&(k._maxInstanceCount=he.meshPerAttribute*he.count)}else for(let ce=0;ce<z.locationSize;ce++)d(z.location+ce);n.bindBuffer(n.ARRAY_BUFFER,je);for(let ce=0;ce<z.locationSize;ce++)v(z.location+ce,_e/z.locationSize,H,ge,_e*Q,_e/z.locationSize*ce*Q,fe)}}else if(V!==void 0){let ge=V[K];if(ge!==void 0)switch(ge.length){case 2:n.vertexAttrib2fv(z.location,ge);break;case 3:n.vertexAttrib3fv(z.location,ge);break;case 4:n.vertexAttrib4fv(z.location,ge);break;default:n.vertexAttrib1fv(z.location,ge)}}}}g()}function R(){L();for(let M in i){let P=i[M];for(let W in P){let k=P[W];for(let G in k)u(k[G].object),delete k[G];delete P[W]}delete i[M]}}function C(M){if(i[M.id]===void 0)return;let P=i[M.id];for(let W in P){let k=P[W];for(let G in k)u(k[G].object),delete k[G];delete P[W]}delete i[M.id]}function T(M){for(let P in i){let W=i[P];if(W[M.id]===void 0)continue;let k=W[M.id];for(let G in k)u(k[G].object),delete k[G];delete W[M.id]}}function L(){E(),o=!0,s!==r&&(s=r,c(s.object))}function E(){r.geometry=null,r.program=null,r.wireframe=!1}return{setup:a,reset:L,resetDefaultState:E,dispose:R,releaseStatesOfGeometry:C,releaseStatesOfProgram:T,initAttributes:y,enableAttribute:d,disableUnusedAttributes:g}}function RE(n,e,t){let i;function r(c){i=c}function s(c,u){n.drawArrays(i,c,u),t.update(u,i,1)}function o(c,u,p){p!==0&&(n.drawArraysInstanced(i,c,u,p),t.update(u,i,p))}function a(c,u,p){if(p===0)return;e.get("WEBGL_multi_draw").multiDrawArraysWEBGL(i,c,0,u,0,p);let m=0;for(let _=0;_<p;_++)m+=u[_];t.update(m,i,1)}function l(c,u,p,f){if(p===0)return;let m=e.get("WEBGL_multi_draw");if(m===null)for(let _=0;_<c.length;_++)o(c[_],u[_],f[_]);else{m.multiDrawArraysInstancedWEBGL(i,c,0,u,0,f,0,p);let _=0;for(let y=0;y<p;y++)_+=u[y];for(let y=0;y<f.length;y++)t.update(_,i,f[y])}}this.setMode=r,this.render=s,this.renderInstances=o,this.renderMultiDraw=a,this.renderMultiDrawInstances=l}function PE(n,e,t,i){let r;function s(){if(r!==void 0)return r;if(e.has("EXT_texture_filter_anisotropic")===!0){let C=e.get("EXT_texture_filter_anisotropic");r=n.getParameter(C.MAX_TEXTURE_MAX_ANISOTROPY_EXT)}else r=0;return r}function o(C){return!(C!==Un&&i.convert(C)!==n.getParameter(n.IMPLEMENTATION_COLOR_READ_FORMAT))}function a(C){let T=C===aa&&(e.has("EXT_color_buffer_half_float")||e.has("EXT_color_buffer_float"));return!(C!==di&&i.convert(C)!==n.getParameter(n.IMPLEMENTATION_COLOR_READ_TYPE)&&C!==ci&&!T)}function l(C){if(C==="highp"){if(n.getShaderPrecisionFormat(n.VERTEX_SHADER,n.HIGH_FLOAT).precision>0&&n.getShaderPrecisionFormat(n.FRAGMENT_SHADER,n.HIGH_FLOAT).precision>0)return"highp";C="mediump"}return C==="mediump"&&n.getShaderPrecisionFormat(n.VERTEX_SHADER,n.MEDIUM_FLOAT).precision>0&&n.getShaderPrecisionFormat(n.FRAGMENT_SHADER,n.MEDIUM_FLOAT).precision>0?"mediump":"lowp"}let c=t.precision!==void 0?t.precision:"highp",u=l(c);u!==c&&(console.warn("THREE.WebGLRenderer:",c,"not supported, using",u,"instead."),c=u);let p=t.logarithmicDepthBuffer===!0,f=n.getParameter(n.MAX_TEXTURE_IMAGE_UNITS),m=n.getParameter(n.MAX_VERTEX_TEXTURE_IMAGE_UNITS),_=n.getParameter(n.MAX_TEXTURE_SIZE),y=n.getParameter(n.MAX_CUBE_MAP_TEXTURE_SIZE),d=n.getParameter(n.MAX_VERTEX_ATTRIBS),h=n.getParameter(n.MAX_VERTEX_UNIFORM_VECTORS),g=n.getParameter(n.MAX_VARYING_VECTORS),v=n.getParameter(n.MAX_FRAGMENT_UNIFORM_VECTORS),x=m>0,R=n.getParameter(n.MAX_SAMPLES);return{isWebGL2:!0,getMaxAnisotropy:s,getMaxPrecision:l,textureFormatReadable:o,textureTypeReadable:a,precision:c,logarithmicDepthBuffer:p,maxTextures:f,maxVertexTextures:m,maxTextureSize:_,maxCubemapSize:y,maxAttributes:d,maxVertexUniforms:h,maxVaryings:g,maxFragmentUniforms:v,vertexTextures:x,maxSamples:R}}function IE(n){let e=this,t=null,i=0,r=!1,s=!1,o=new Ln,a=new Ue,l={value:null,needsUpdate:!1};this.uniform=l,this.numPlanes=0,this.numIntersection=0,this.init=function(p,f){let m=p.length!==0||f||i!==0||r;return r=f,i=p.length,m},this.beginShadows=function(){s=!0,u(null)},this.endShadows=function(){s=!1},this.setGlobalState=function(p,f){t=u(p,f,0)},this.setState=function(p,f,m){let _=p.clippingPlanes,y=p.clipIntersection,d=p.clipShadows,h=n.get(p);if(!r||_===null||_.length===0||s&&!d)s?u(null):c();else{let g=s?0:i,v=g*4,x=h.clippingState||null;l.value=x,x=u(_,f,v,m);for(let R=0;R!==v;++R)x[R]=t[R];h.clippingState=x,this.numIntersection=y?this.numPlanes:0,this.numPlanes+=g}};function c(){l.value!==t&&(l.value=t,l.needsUpdate=i>0),e.numPlanes=i,e.numIntersection=0}function u(p,f,m,_){let y=p!==null?p.length:0,d=null;if(y!==0){if(d=l.value,_!==!0||d===null){let h=m+y*4,g=f.matrixWorldInverse;a.getNormalMatrix(g),(d===null||d.length<h)&&(d=new Float32Array(h));for(let v=0,x=m;v!==y;++v,x+=4)o.copy(p[v]).applyMatrix4(g,a),o.normal.toArray(d,x),d[x+3]=o.constant}l.value=d,l.needsUpdate=!0}return e.numPlanes=y,e.numIntersection=0,d}}function LE(n){let e=new WeakMap;function t(o,a){return a===rf?o.mapping=Fs:a===sf&&(o.mapping=Bs),o}function i(o){if(o&&o.isTexture){let a=o.mapping;if(a===rf||a===sf)if(e.has(o)){let l=e.get(o).texture;return t(l,o.mapping)}else{let l=o.image;if(l&&l.height>0){let c=new Vf(l.height);return c.fromEquirectangularTexture(n,o),e.set(o,c),o.addEventListener("dispose",r),t(c.texture,o.mapping)}else return null}}return o}function r(o){let a=o.target;a.removeEventListener("dispose",r);let l=e.get(a);l!==void 0&&(e.delete(a),l.dispose())}function s(){e=new WeakMap}return{get:i,dispose:s}}var yc=class extends _c{constructor(e=-1,t=1,i=1,r=-1,s=.1,o=2e3){super(),this.isOrthographicCamera=!0,this.type="OrthographicCamera",this.zoom=1,this.view=null,this.left=e,this.right=t,this.top=i,this.bottom=r,this.near=s,this.far=o,this.updateProjectionMatrix()}copy(e,t){return super.copy(e,t),this.left=e.left,this.right=e.right,this.top=e.top,this.bottom=e.bottom,this.near=e.near,this.far=e.far,this.zoom=e.zoom,this.view=e.view===null?null:Object.assign({},e.view),this}setViewOffset(e,t,i,r,s,o){this.view===null&&(this.view={enabled:!0,fullWidth:1,fullHeight:1,offsetX:0,offsetY:0,width:1,height:1}),this.view.enabled=!0,this.view.fullWidth=e,this.view.fullHeight=t,this.view.offsetX=i,this.view.offsetY=r,this.view.width=s,this.view.height=o,this.updateProjectionMatrix()}clearViewOffset(){this.view!==null&&(this.view.enabled=!1),this.updateProjectionMatrix()}updateProjectionMatrix(){let e=(this.right-this.left)/(2*this.zoom),t=(this.top-this.bottom)/(2*this.zoom),i=(this.right+this.left)/2,r=(this.top+this.bottom)/2,s=i-e,o=i+e,a=r+t,l=r-t;if(this.view!==null&&this.view.enabled){let c=(this.right-this.left)/this.view.fullWidth/this.zoom,u=(this.top-this.bottom)/this.view.fullHeight/this.zoom;s+=c*this.view.offsetX,o=s+c*this.view.width,a-=u*this.view.offsetY,l=a-u*this.view.height}this.projectionMatrix.makeOrthographic(s,o,a,l,this.near,this.far,this.coordinateSystem),this.projectionMatrixInverse.copy(this.projectionMatrix).invert()}toJSON(e){let t=super.toJSON(e);return t.object.zoom=this.zoom,t.object.left=this.left,t.object.right=this.right,t.object.top=this.top,t.object.bottom=this.bottom,t.object.near=this.near,t.object.far=this.far,this.view!==null&&(t.object.view=Object.assign({},this.view)),t}},Ls=4,W_=[.125,.215,.35,.446,.526,.582],Mr=20,$d=new yc,X_=new Oe,Zd=null,Jd=0,Kd=0,jd=!1,wr=(1+Math.sqrt(5))/2,Ps=1/wr,Y_=[new D(-wr,Ps,0),new D(wr,Ps,0),new D(-Ps,0,wr),new D(Ps,0,wr),new D(0,wr,-Ps),new D(0,wr,Ps),new D(-1,1,-1),new D(1,1,-1),new D(-1,1,1),new D(1,1,1)],xc=class{constructor(e){this._renderer=e,this._pingPongRenderTarget=null,this._lodMax=0,this._cubeSize=0,this._lodPlanes=[],this._sizeLods=[],this._sigmas=[],this._blurMaterial=null,this._cubemapMaterial=null,this._equirectMaterial=null,this._compileMaterial(this._blurMaterial)}fromScene(e,t=0,i=.1,r=100){Zd=this._renderer.getRenderTarget(),Jd=this._renderer.getActiveCubeFace(),Kd=this._renderer.getActiveMipmapLevel(),jd=this._renderer.xr.enabled,this._renderer.xr.enabled=!1,this._setSize(256);let s=this._allocateTargets();return s.depthBuffer=!0,this._sceneToCubeUV(e,i,r,s),t>0&&this._blur(s,0,0,t),this._applyPMREM(s),this._cleanup(s),s}fromEquirectangular(e,t=null){return this._fromTexture(e,t)}fromCubemap(e,t=null){return this._fromTexture(e,t)}compileCubemapShader(){this._cubemapMaterial===null&&(this._cubemapMaterial=Z_(),this._compileMaterial(this._cubemapMaterial))}compileEquirectangularShader(){this._equirectMaterial===null&&(this._equirectMaterial=$_(),this._compileMaterial(this._equirectMaterial))}dispose(){this._dispose(),this._cubemapMaterial!==null&&this._cubemapMaterial.dispose(),this._equirectMaterial!==null&&this._equirectMaterial.dispose()}_setSize(e){this._lodMax=Math.floor(Math.log2(e)),this._cubeSize=Math.pow(2,this._lodMax)}_dispose(){this._blurMaterial!==null&&this._blurMaterial.dispose(),this._pingPongRenderTarget!==null&&this._pingPongRenderTarget.dispose();for(let e=0;e<this._lodPlanes.length;e++)this._lodPlanes[e].dispose()}_cleanup(e){this._renderer.setRenderTarget(Zd,Jd,Kd),this._renderer.xr.enabled=jd,e.scissorTest=!1,Ql(e,0,0,e.width,e.height)}_fromTexture(e,t){e.mapping===Fs||e.mapping===Bs?this._setSize(e.image.length===0?16:e.image[0].width||e.image[0].image.width):this._setSize(e.image.width/4),Zd=this._renderer.getRenderTarget(),Jd=this._renderer.getActiveCubeFace(),Kd=this._renderer.getActiveMipmapLevel(),jd=this._renderer.xr.enabled,this._renderer.xr.enabled=!1;let i=t||this._allocateTargets();return this._textureToCubeUV(e,i),this._applyPMREM(i),this._cleanup(i),i}_allocateTargets(){let e=3*Math.max(this._cubeSize,112),t=4*this._cubeSize,i={magFilter:qt,minFilter:qt,generateMipmaps:!1,type:aa,format:Un,colorSpace:Ji,depthBuffer:!1},r=q_(e,t,i);if(this._pingPongRenderTarget===null||this._pingPongRenderTarget.width!==e||this._pingPongRenderTarget.height!==t){this._pingPongRenderTarget!==null&&this._dispose(),this._pingPongRenderTarget=q_(e,t,i);let{_lodMax:s}=this;({sizeLods:this._sizeLods,lodPlanes:this._lodPlanes,sigmas:this._sigmas}=UE(s)),this._blurMaterial=NE(s,e,t)}return r}_compileMaterial(e){let t=new St(this._lodPlanes[0],e);this._renderer.compile(t,$d)}_sceneToCubeUV(e,t,i,r){let a=new Yt(90,1,t,i),l=[1,-1,1,1,1,1],c=[1,1,1,-1,-1,-1],u=this._renderer,p=u.autoClear,f=u.toneMapping;u.getClearColor(X_),u.toneMapping=qi,u.autoClear=!1;let m=new Rr({name:"PMREM.Background",side:tn,depthWrite:!1,depthTest:!1}),_=new St(new Pr,m),y=!1,d=e.background;d?d.isColor&&(m.color.copy(d),e.background=null,y=!0):(m.color.copy(X_),y=!0);for(let h=0;h<6;h++){let g=h%3;g===0?(a.up.set(0,l[h],0),a.lookAt(c[h],0,0)):g===1?(a.up.set(0,0,l[h]),a.lookAt(0,c[h],0)):(a.up.set(0,l[h],0),a.lookAt(0,0,c[h]));let v=this._cubeSize;Ql(r,g*v,h>2?v:0,v,v),u.setRenderTarget(r),y&&u.render(_,a),u.render(e,a)}_.geometry.dispose(),_.material.dispose(),u.toneMapping=f,u.autoClear=p,e.background=d}_textureToCubeUV(e,t){let i=this._renderer,r=e.mapping===Fs||e.mapping===Bs;r?(this._cubemapMaterial===null&&(this._cubemapMaterial=Z_()),this._cubemapMaterial.uniforms.flipEnvMap.value=e.isRenderTargetTexture===!1?-1:1):this._equirectMaterial===null&&(this._equirectMaterial=$_());let s=r?this._cubemapMaterial:this._equirectMaterial,o=new St(this._lodPlanes[0],s),a=s.uniforms;a.envMap.value=e;let l=this._cubeSize;Ql(t,0,0,3*l,2*l),i.setRenderTarget(t),i.render(o,$d)}_applyPMREM(e){let t=this._renderer,i=t.autoClear;t.autoClear=!1;let r=this._lodPlanes.length;for(let s=1;s<r;s++){let o=Math.sqrt(this._sigmas[s]*this._sigmas[s]-this._sigmas[s-1]*this._sigmas[s-1]),a=Y_[(r-s-1)%Y_.length];this._blur(e,s-1,s,o,a)}t.autoClear=i}_blur(e,t,i,r,s){let o=this._pingPongRenderTarget;this._halfBlur(e,o,t,i,r,"latitudinal",s),this._halfBlur(o,e,i,i,r,"longitudinal",s)}_halfBlur(e,t,i,r,s,o,a){let l=this._renderer,c=this._blurMaterial;o!=="latitudinal"&&o!=="longitudinal"&&console.error("blur direction must be either latitudinal or longitudinal!");let u=3,p=new St(this._lodPlanes[r],c),f=c.uniforms,m=this._sizeLods[i]-1,_=isFinite(s)?Math.PI/(2*m):2*Math.PI/(2*Mr-1),y=s/_,d=isFinite(s)?1+Math.floor(u*y):Mr;d>Mr&&console.warn(`sigmaRadians, ${s}, is too large and will clip, as it requested ${d} samples when the maximum is set to ${Mr}`);let h=[],g=0;for(let T=0;T<Mr;++T){let L=T/y,E=Math.exp(-L*L/2);h.push(E),T===0?g+=E:T<d&&(g+=2*E)}for(let T=0;T<h.length;T++)h[T]=h[T]/g;f.envMap.value=e.texture,f.samples.value=d,f.weights.value=h,f.latitudinal.value=o==="latitudinal",a&&(f.poleAxis.value=a);let{_lodMax:v}=this;f.dTheta.value=_,f.mipInt.value=v-i;let x=this._sizeLods[r],R=3*x*(r>v-Ls?r-v+Ls:0),C=4*(this._cubeSize-x);Ql(t,R,C,3*x,2*x),l.setRenderTarget(t),l.render(p,$d)}};function UE(n){let e=[],t=[],i=[],r=n,s=n-Ls+1+W_.length;for(let o=0;o<s;o++){let a=Math.pow(2,r);t.push(a);let l=1/a;o>n-Ls?l=W_[o-n+Ls-1]:o===0&&(l=0),i.push(l);let c=1/(a-2),u=-c,p=1+c,f=[u,u,p,u,p,p,u,u,p,p,u,p],m=6,_=6,y=3,d=2,h=1,g=new Float32Array(y*_*m),v=new Float32Array(d*_*m),x=new Float32Array(h*_*m);for(let C=0;C<m;C++){let T=C%3*2/3-1,L=C>2?0:-1,E=[T,L,0,T+2/3,L,0,T+2/3,L+1,0,T,L,0,T+2/3,L+1,0,T,L+1,0];g.set(E,y*_*C),v.set(f,d*_*C);let M=[C,C,C,C,C,C];x.set(M,h*_*C)}let R=new pi;R.setAttribute("position",new nn(g,y)),R.setAttribute("uv",new nn(v,d)),R.setAttribute("faceIndex",new nn(x,h)),e.push(R),r>Ls&&r--}return{lodPlanes:e,sizeLods:t,sigmas:i}}function q_(n,e,t){let i=new fi(n,e,t);return i.texture.mapping=Lc,i.texture.name="PMREM.cubeUv",i.scissorTest=!0,i}function Ql(n,e,t,i,r){n.viewport.set(e,t,i,r),n.scissor.set(e,t,i,r)}function NE(n,e,t){let i=new Float32Array(Mr),r=new D(0,1,0);return new Mn({name:"SphericalGaussianBlur",defines:{n:Mr,CUBEUV_TEXEL_WIDTH:1/e,CUBEUV_TEXEL_HEIGHT:1/t,CUBEUV_MAX_MIP:`${n}.0`},uniforms:{envMap:{value:null},samples:{value:1},weights:{value:i},latitudinal:{value:!1},dTheta:{value:0},mipInt:{value:0},poleAxis:{value:r}},vertexShader:wp(),fragmentShader:`

			precision mediump float;
			precision mediump int;

			varying vec3 vOutputDirection;

			uniform sampler2D envMap;
			uniform int samples;
			uniform float weights[ n ];
			uniform bool latitudinal;
			uniform float dTheta;
			uniform float mipInt;
			uniform vec3 poleAxis;

			#define ENVMAP_TYPE_CUBE_UV
			#include <cube_uv_reflection_fragment>

			vec3 getSample( float theta, vec3 axis ) {

				float cosTheta = cos( theta );
				// Rodrigues' axis-angle rotation
				vec3 sampleDirection = vOutputDirection * cosTheta
					+ cross( axis, vOutputDirection ) * sin( theta )
					+ axis * dot( axis, vOutputDirection ) * ( 1.0 - cosTheta );

				return bilinearCubeUV( envMap, sampleDirection, mipInt );

			}

			void main() {

				vec3 axis = latitudinal ? poleAxis : cross( poleAxis, vOutputDirection );

				if ( all( equal( axis, vec3( 0.0 ) ) ) ) {

					axis = vec3( vOutputDirection.z, 0.0, - vOutputDirection.x );

				}

				axis = normalize( axis );

				gl_FragColor = vec4( 0.0, 0.0, 0.0, 1.0 );
				gl_FragColor.rgb += weights[ 0 ] * getSample( 0.0, axis );

				for ( int i = 1; i < n; i++ ) {

					if ( i >= samples ) {

						break;

					}

					float theta = dTheta * float( i );
					gl_FragColor.rgb += weights[ i ] * getSample( -1.0 * theta, axis );
					gl_FragColor.rgb += weights[ i ] * getSample( theta, axis );

				}

			}
		`,blending:Yi,depthTest:!1,depthWrite:!1})}function $_(){return new Mn({name:"EquirectangularToCubeUV",uniforms:{envMap:{value:null}},vertexShader:wp(),fragmentShader:`

			precision mediump float;
			precision mediump int;

			varying vec3 vOutputDirection;

			uniform sampler2D envMap;

			#include <common>

			void main() {

				vec3 outputDirection = normalize( vOutputDirection );
				vec2 uv = equirectUv( outputDirection );

				gl_FragColor = vec4( texture2D ( envMap, uv ).rgb, 1.0 );

			}
		`,blending:Yi,depthTest:!1,depthWrite:!1})}function Z_(){return new Mn({name:"CubemapToCubeUV",uniforms:{envMap:{value:null},flipEnvMap:{value:-1}},vertexShader:wp(),fragmentShader:`

			precision mediump float;
			precision mediump int;

			uniform float flipEnvMap;

			varying vec3 vOutputDirection;

			uniform samplerCube envMap;

			void main() {

				gl_FragColor = textureCube( envMap, vec3( flipEnvMap * vOutputDirection.x, vOutputDirection.yz ) );

			}
		`,blending:Yi,depthTest:!1,depthWrite:!1})}function wp(){return`

		precision mediump float;
		precision mediump int;

		attribute float faceIndex;

		varying vec3 vOutputDirection;

		// RH coordinate system; PMREM face-indexing convention
		vec3 getDirection( vec2 uv, float face ) {

			uv = 2.0 * uv - 1.0;

			vec3 direction = vec3( uv, 1.0 );

			if ( face == 0.0 ) {

				direction = direction.zyx; // ( 1, v, u ) pos x

			} else if ( face == 1.0 ) {

				direction = direction.xzy;
				direction.xz *= -1.0; // ( -u, 1, -v ) pos y

			} else if ( face == 2.0 ) {

				direction.x *= -1.0; // ( -u, v, 1 ) pos z

			} else if ( face == 3.0 ) {

				direction = direction.zyx;
				direction.xz *= -1.0; // ( -1, v, -u ) neg x

			} else if ( face == 4.0 ) {

				direction = direction.xzy;
				direction.xy *= -1.0; // ( -u, -1, v ) neg y

			} else if ( face == 5.0 ) {

				direction.z *= -1.0; // ( u, v, -1 ) neg z

			}

			return direction;

		}

		void main() {

			vOutputDirection = getDirection( uv, faceIndex );
			gl_Position = vec4( position, 1.0 );

		}
	`}function DE(n){let e=new WeakMap,t=null;function i(a){if(a&&a.isTexture){let l=a.mapping,c=l===rf||l===sf,u=l===Fs||l===Bs;if(c||u){let p=e.get(a),f=p!==void 0?p.texture.pmremVersion:0;if(a.isRenderTargetTexture&&a.pmremVersion!==f)return t===null&&(t=new xc(n)),p=c?t.fromEquirectangular(a,p):t.fromCubemap(a,p),p.texture.pmremVersion=a.pmremVersion,e.set(a,p),p.texture;if(p!==void 0)return p.texture;{let m=a.image;return c&&m&&m.height>0||u&&m&&r(m)?(t===null&&(t=new xc(n)),p=c?t.fromEquirectangular(a):t.fromCubemap(a),p.texture.pmremVersion=a.pmremVersion,e.set(a,p),a.addEventListener("dispose",s),p.texture):null}}}return a}function r(a){let l=0,c=6;for(let u=0;u<c;u++)a[u]!==void 0&&l++;return l===c}function s(a){let l=a.target;l.removeEventListener("dispose",s);let c=e.get(l);c!==void 0&&(e.delete(l),c.dispose())}function o(){e=new WeakMap,t!==null&&(t.dispose(),t=null)}return{get:i,dispose:o}}function OE(n){let e={};function t(i){if(e[i]!==void 0)return e[i];let r;switch(i){case"WEBGL_depth_texture":r=n.getExtension("WEBGL_depth_texture")||n.getExtension("MOZ_WEBGL_depth_texture")||n.getExtension("WEBKIT_WEBGL_depth_texture");break;case"EXT_texture_filter_anisotropic":r=n.getExtension("EXT_texture_filter_anisotropic")||n.getExtension("MOZ_EXT_texture_filter_anisotropic")||n.getExtension("WEBKIT_EXT_texture_filter_anisotropic");break;case"WEBGL_compressed_texture_s3tc":r=n.getExtension("WEBGL_compressed_texture_s3tc")||n.getExtension("MOZ_WEBGL_compressed_texture_s3tc")||n.getExtension("WEBKIT_WEBGL_compressed_texture_s3tc");break;case"WEBGL_compressed_texture_pvrtc":r=n.getExtension("WEBGL_compressed_texture_pvrtc")||n.getExtension("WEBKIT_WEBGL_compressed_texture_pvrtc");break;default:r=n.getExtension(i)}return e[i]=r,r}return{has:function(i){return t(i)!==null},init:function(){t("EXT_color_buffer_float"),t("WEBGL_clip_cull_distance"),t("OES_texture_float_linear"),t("EXT_color_buffer_half_float"),t("WEBGL_multisampled_render_to_texture"),t("WEBGL_render_shared_exponent")},get:function(i){let r=t(i);return r===null&&Iv("THREE.WebGLRenderer: "+i+" extension not supported."),r}}}function FE(n,e,t,i){let r={},s=new WeakMap;function o(p){let f=p.target;f.index!==null&&e.remove(f.index);for(let _ in f.attributes)e.remove(f.attributes[_]);for(let _ in f.morphAttributes){let y=f.morphAttributes[_];for(let d=0,h=y.length;d<h;d++)e.remove(y[d])}f.removeEventListener("dispose",o),delete r[f.id];let m=s.get(f);m&&(e.remove(m),s.delete(f)),i.releaseStatesOfGeometry(f),f.isInstancedBufferGeometry===!0&&delete f._maxInstanceCount,t.memory.geometries--}function a(p,f){return r[f.id]===!0||(f.addEventListener("dispose",o),r[f.id]=!0,t.memory.geometries++),f}function l(p){let f=p.attributes;for(let _ in f)e.update(f[_],n.ARRAY_BUFFER);let m=p.morphAttributes;for(let _ in m){let y=m[_];for(let d=0,h=y.length;d<h;d++)e.update(y[d],n.ARRAY_BUFFER)}}function c(p){let f=[],m=p.index,_=p.attributes.position,y=0;if(m!==null){let g=m.array;y=m.version;for(let v=0,x=g.length;v<x;v+=3){let R=g[v+0],C=g[v+1],T=g[v+2];f.push(R,C,C,T,T,R)}}else if(_!==void 0){let g=_.array;y=_.version;for(let v=0,x=g.length/3-1;v<x;v+=3){let R=v+0,C=v+1,T=v+2;f.push(R,C,C,T,T,R)}}else return;let d=new(Pv(f)?gc:mc)(f,1);d.version=y;let h=s.get(p);h&&e.remove(h),s.set(p,d)}function u(p){let f=s.get(p);if(f){let m=p.index;m!==null&&f.version<m.version&&c(p)}else c(p);return s.get(p)}return{get:a,update:l,getWireframeAttribute:u}}function BE(n,e,t){let i;function r(f){i=f}let s,o;function a(f){s=f.type,o=f.bytesPerElement}function l(f,m){n.drawElements(i,m,s,f*o),t.update(m,i,1)}function c(f,m,_){_!==0&&(n.drawElementsInstanced(i,m,s,f*o,_),t.update(m,i,_))}function u(f,m,_){if(_===0)return;e.get("WEBGL_multi_draw").multiDrawElementsWEBGL(i,m,0,s,f,0,_);let d=0;for(let h=0;h<_;h++)d+=m[h];t.update(d,i,1)}function p(f,m,_,y){if(_===0)return;let d=e.get("WEBGL_multi_draw");if(d===null)for(let h=0;h<f.length;h++)c(f[h]/o,m[h],y[h]);else{d.multiDrawElementsInstancedWEBGL(i,m,0,s,f,0,y,0,_);let h=0;for(let g=0;g<_;g++)h+=m[g];for(let g=0;g<y.length;g++)t.update(h,i,y[g])}}this.setMode=r,this.setIndex=a,this.render=l,this.renderInstances=c,this.renderMultiDraw=u,this.renderMultiDrawInstances=p}function kE(n){let e={geometries:0,textures:0},t={frame:0,calls:0,triangles:0,points:0,lines:0};function i(s,o,a){switch(t.calls++,o){case n.TRIANGLES:t.triangles+=a*(s/3);break;case n.LINES:t.lines+=a*(s/2);break;case n.LINE_STRIP:t.lines+=a*(s-1);break;case n.LINE_LOOP:t.lines+=a*s;break;case n.POINTS:t.points+=a*s;break;default:console.error("THREE.WebGLInfo: Unknown draw mode:",o);break}}function r(){t.calls=0,t.triangles=0,t.points=0,t.lines=0}return{memory:e,render:t,programs:null,autoReset:!0,reset:r,update:i}}function zE(n,e,t){let i=new WeakMap,r=new wt;function s(o,a,l){let c=o.morphTargetInfluences,u=a.morphAttributes.position||a.morphAttributes.normal||a.morphAttributes.color,p=u!==void 0?u.length:0,f=i.get(a);if(f===void 0||f.count!==p){let E=function(){T.dispose(),i.delete(a),a.removeEventListener("dispose",E)};f!==void 0&&f.texture.dispose();let m=a.morphAttributes.position!==void 0,_=a.morphAttributes.normal!==void 0,y=a.morphAttributes.color!==void 0,d=a.morphAttributes.position||[],h=a.morphAttributes.normal||[],g=a.morphAttributes.color||[],v=0;m===!0&&(v=1),_===!0&&(v=2),y===!0&&(v=3);let x=a.attributes.position.count*v,R=1;x>e.maxTextureSize&&(R=Math.ceil(x/e.maxTextureSize),x=e.maxTextureSize);let C=new Float32Array(x*R*4*p),T=new fc(C,x,R,p);T.type=ci,T.needsUpdate=!0;let L=v*4;for(let M=0;M<p;M++){let P=d[M],W=h[M],k=g[M],G=x*R*4*M;for(let $=0;$<P.count;$++){let V=$*L;m===!0&&(r.fromBufferAttribute(P,$),C[G+V+0]=r.x,C[G+V+1]=r.y,C[G+V+2]=r.z,C[G+V+3]=0),_===!0&&(r.fromBufferAttribute(W,$),C[G+V+4]=r.x,C[G+V+5]=r.y,C[G+V+6]=r.z,C[G+V+7]=0),y===!0&&(r.fromBufferAttribute(k,$),C[G+V+8]=r.x,C[G+V+9]=r.y,C[G+V+10]=r.z,C[G+V+11]=k.itemSize===4?r.w:1)}}f={count:p,texture:T,size:new ze(x,R)},i.set(a,f),a.addEventListener("dispose",E)}if(o.isInstancedMesh===!0&&o.morphTexture!==null)l.getUniforms().setValue(n,"morphTexture",o.morphTexture,t);else{let m=0;for(let y=0;y<c.length;y++)m+=c[y];let _=a.morphTargetsRelative?1:1-m;l.getUniforms().setValue(n,"morphTargetBaseInfluence",_),l.getUniforms().setValue(n,"morphTargetInfluences",c)}l.getUniforms().setValue(n,"morphTargetsTexture",f.texture,t),l.getUniforms().setValue(n,"morphTargetsTextureSize",f.size)}return{update:s}}function VE(n,e,t,i){let r=new WeakMap;function s(l){let c=i.render.frame,u=l.geometry,p=e.get(l,u);if(r.get(p)!==c&&(e.update(p),r.set(p,c)),l.isInstancedMesh&&(l.hasEventListener("dispose",a)===!1&&l.addEventListener("dispose",a),r.get(l)!==c&&(t.update(l.instanceMatrix,n.ARRAY_BUFFER),l.instanceColor!==null&&t.update(l.instanceColor,n.ARRAY_BUFFER),r.set(l,c))),l.isSkinnedMesh){let f=l.skeleton;r.get(f)!==c&&(f.update(),r.set(f,c))}return p}function o(){r=new WeakMap}function a(l){let c=l.target;c.removeEventListener("dispose",a),t.remove(c.instanceMatrix),c.instanceColor!==null&&t.remove(c.instanceColor)}return{update:s,dispose:o}}var wc=class extends $t{constructor(e,t,i,r,s,o,a,l,c,u=Ns){if(u!==Ns&&u!==zs)throw new Error("DepthTexture format must be either THREE.DepthFormat or THREE.DepthStencilFormat");i===void 0&&u===Ns&&(i=Tr),i===void 0&&u===zs&&(i=ks),super(null,r,s,o,a,l,u,i,c),this.isDepthTexture=!0,this.image={width:e,height:t},this.magFilter=a!==void 0?a:Sn,this.minFilter=l!==void 0?l:Sn,this.flipY=!1,this.generateMipmaps=!1,this.compareFunction=null}copy(e){return super.copy(e),this.compareFunction=e.compareFunction,this}toJSON(e){let t=super.toJSON(e);return this.compareFunction!==null&&(t.compareFunction=this.compareFunction),t}},Dv=new $t,J_=new wc(1,1),Ov=new fc,Fv=new Bf,Bv=new vc,K_=[],j_=[],Q_=new Float32Array(16),ev=new Float32Array(9),tv=new Float32Array(4);function Ws(n,e,t){let i=n[0];if(i<=0||i>0)return n;let r=e*t,s=K_[r];if(s===void 0&&(s=new Float32Array(r),K_[r]=s),e!==0){i.toArray(s,0);for(let o=1,a=0;o!==e;++o)a+=t,n[o].toArray(s,a)}return s}function Mt(n,e){if(n.length!==e.length)return!1;for(let t=0,i=n.length;t<i;t++)if(n[t]!==e[t])return!1;return!0}function Et(n,e){for(let t=0,i=e.length;t<i;t++)n[t]=e[t]}function Nc(n,e){let t=j_[e];t===void 0&&(t=new Int32Array(e),j_[e]=t);for(let i=0;i!==e;++i)t[i]=n.allocateTextureUnit();return t}function HE(n,e){let t=this.cache;t[0]!==e&&(n.uniform1f(this.addr,e),t[0]=e)}function GE(n,e){let t=this.cache;if(e.x!==void 0)(t[0]!==e.x||t[1]!==e.y)&&(n.uniform2f(this.addr,e.x,e.y),t[0]=e.x,t[1]=e.y);else{if(Mt(t,e))return;n.uniform2fv(this.addr,e),Et(t,e)}}function WE(n,e){let t=this.cache;if(e.x!==void 0)(t[0]!==e.x||t[1]!==e.y||t[2]!==e.z)&&(n.uniform3f(this.addr,e.x,e.y,e.z),t[0]=e.x,t[1]=e.y,t[2]=e.z);else if(e.r!==void 0)(t[0]!==e.r||t[1]!==e.g||t[2]!==e.b)&&(n.uniform3f(this.addr,e.r,e.g,e.b),t[0]=e.r,t[1]=e.g,t[2]=e.b);else{if(Mt(t,e))return;n.uniform3fv(this.addr,e),Et(t,e)}}function XE(n,e){let t=this.cache;if(e.x!==void 0)(t[0]!==e.x||t[1]!==e.y||t[2]!==e.z||t[3]!==e.w)&&(n.uniform4f(this.addr,e.x,e.y,e.z,e.w),t[0]=e.x,t[1]=e.y,t[2]=e.z,t[3]=e.w);else{if(Mt(t,e))return;n.uniform4fv(this.addr,e),Et(t,e)}}function YE(n,e){let t=this.cache,i=e.elements;if(i===void 0){if(Mt(t,e))return;n.uniformMatrix2fv(this.addr,!1,e),Et(t,e)}else{if(Mt(t,i))return;tv.set(i),n.uniformMatrix2fv(this.addr,!1,tv),Et(t,i)}}function qE(n,e){let t=this.cache,i=e.elements;if(i===void 0){if(Mt(t,e))return;n.uniformMatrix3fv(this.addr,!1,e),Et(t,e)}else{if(Mt(t,i))return;ev.set(i),n.uniformMatrix3fv(this.addr,!1,ev),Et(t,i)}}function $E(n,e){let t=this.cache,i=e.elements;if(i===void 0){if(Mt(t,e))return;n.uniformMatrix4fv(this.addr,!1,e),Et(t,e)}else{if(Mt(t,i))return;Q_.set(i),n.uniformMatrix4fv(this.addr,!1,Q_),Et(t,i)}}function ZE(n,e){let t=this.cache;t[0]!==e&&(n.uniform1i(this.addr,e),t[0]=e)}function JE(n,e){let t=this.cache;if(e.x!==void 0)(t[0]!==e.x||t[1]!==e.y)&&(n.uniform2i(this.addr,e.x,e.y),t[0]=e.x,t[1]=e.y);else{if(Mt(t,e))return;n.uniform2iv(this.addr,e),Et(t,e)}}function KE(n,e){let t=this.cache;if(e.x!==void 0)(t[0]!==e.x||t[1]!==e.y||t[2]!==e.z)&&(n.uniform3i(this.addr,e.x,e.y,e.z),t[0]=e.x,t[1]=e.y,t[2]=e.z);else{if(Mt(t,e))return;n.uniform3iv(this.addr,e),Et(t,e)}}function jE(n,e){let t=this.cache;if(e.x!==void 0)(t[0]!==e.x||t[1]!==e.y||t[2]!==e.z||t[3]!==e.w)&&(n.uniform4i(this.addr,e.x,e.y,e.z,e.w),t[0]=e.x,t[1]=e.y,t[2]=e.z,t[3]=e.w);else{if(Mt(t,e))return;n.uniform4iv(this.addr,e),Et(t,e)}}function QE(n,e){let t=this.cache;t[0]!==e&&(n.uniform1ui(this.addr,e),t[0]=e)}function eC(n,e){let t=this.cache;if(e.x!==void 0)(t[0]!==e.x||t[1]!==e.y)&&(n.uniform2ui(this.addr,e.x,e.y),t[0]=e.x,t[1]=e.y);else{if(Mt(t,e))return;n.uniform2uiv(this.addr,e),Et(t,e)}}function tC(n,e){let t=this.cache;if(e.x!==void 0)(t[0]!==e.x||t[1]!==e.y||t[2]!==e.z)&&(n.uniform3ui(this.addr,e.x,e.y,e.z),t[0]=e.x,t[1]=e.y,t[2]=e.z);else{if(Mt(t,e))return;n.uniform3uiv(this.addr,e),Et(t,e)}}function nC(n,e){let t=this.cache;if(e.x!==void 0)(t[0]!==e.x||t[1]!==e.y||t[2]!==e.z||t[3]!==e.w)&&(n.uniform4ui(this.addr,e.x,e.y,e.z,e.w),t[0]=e.x,t[1]=e.y,t[2]=e.z,t[3]=e.w);else{if(Mt(t,e))return;n.uniform4uiv(this.addr,e),Et(t,e)}}function iC(n,e,t){let i=this.cache,r=t.allocateTextureUnit();i[0]!==r&&(n.uniform1i(this.addr,r),i[0]=r);let s;this.type===n.SAMPLER_2D_SHADOW?(J_.compareFunction=Rv,s=J_):s=Dv,t.setTexture2D(e||s,r)}function rC(n,e,t){let i=this.cache,r=t.allocateTextureUnit();i[0]!==r&&(n.uniform1i(this.addr,r),i[0]=r),t.setTexture3D(e||Fv,r)}function sC(n,e,t){let i=this.cache,r=t.allocateTextureUnit();i[0]!==r&&(n.uniform1i(this.addr,r),i[0]=r),t.setTextureCube(e||Bv,r)}function oC(n,e,t){let i=this.cache,r=t.allocateTextureUnit();i[0]!==r&&(n.uniform1i(this.addr,r),i[0]=r),t.setTexture2DArray(e||Ov,r)}function aC(n){switch(n){case 5126:return HE;case 35664:return GE;case 35665:return WE;case 35666:return XE;case 35674:return YE;case 35675:return qE;case 35676:return $E;case 5124:case 35670:return ZE;case 35667:case 35671:return JE;case 35668:case 35672:return KE;case 35669:case 35673:return jE;case 5125:return QE;case 36294:return eC;case 36295:return tC;case 36296:return nC;case 35678:case 36198:case 36298:case 36306:case 35682:return iC;case 35679:case 36299:case 36307:return rC;case 35680:case 36300:case 36308:case 36293:return sC;case 36289:case 36303:case 36311:case 36292:return oC}}function lC(n,e){n.uniform1fv(this.addr,e)}function cC(n,e){let t=Ws(e,this.size,2);n.uniform2fv(this.addr,t)}function uC(n,e){let t=Ws(e,this.size,3);n.uniform3fv(this.addr,t)}function hC(n,e){let t=Ws(e,this.size,4);n.uniform4fv(this.addr,t)}function dC(n,e){let t=Ws(e,this.size,4);n.uniformMatrix2fv(this.addr,!1,t)}function fC(n,e){let t=Ws(e,this.size,9);n.uniformMatrix3fv(this.addr,!1,t)}function pC(n,e){let t=Ws(e,this.size,16);n.uniformMatrix4fv(this.addr,!1,t)}function mC(n,e){n.uniform1iv(this.addr,e)}function gC(n,e){n.uniform2iv(this.addr,e)}function _C(n,e){n.uniform3iv(this.addr,e)}function vC(n,e){n.uniform4iv(this.addr,e)}function yC(n,e){n.uniform1uiv(this.addr,e)}function xC(n,e){n.uniform2uiv(this.addr,e)}function wC(n,e){n.uniform3uiv(this.addr,e)}function SC(n,e){n.uniform4uiv(this.addr,e)}function MC(n,e,t){let i=this.cache,r=e.length,s=Nc(t,r);Mt(i,s)||(n.uniform1iv(this.addr,s),Et(i,s));for(let o=0;o!==r;++o)t.setTexture2D(e[o]||Dv,s[o])}function EC(n,e,t){let i=this.cache,r=e.length,s=Nc(t,r);Mt(i,s)||(n.uniform1iv(this.addr,s),Et(i,s));for(let o=0;o!==r;++o)t.setTexture3D(e[o]||Fv,s[o])}function CC(n,e,t){let i=this.cache,r=e.length,s=Nc(t,r);Mt(i,s)||(n.uniform1iv(this.addr,s),Et(i,s));for(let o=0;o!==r;++o)t.setTextureCube(e[o]||Bv,s[o])}function TC(n,e,t){let i=this.cache,r=e.length,s=Nc(t,r);Mt(i,s)||(n.uniform1iv(this.addr,s),Et(i,s));for(let o=0;o!==r;++o)t.setTexture2DArray(e[o]||Ov,s[o])}function bC(n){switch(n){case 5126:return lC;case 35664:return cC;case 35665:return uC;case 35666:return hC;case 35674:return dC;case 35675:return fC;case 35676:return pC;case 5124:case 35670:return mC;case 35667:case 35671:return gC;case 35668:case 35672:return _C;case 35669:case 35673:return vC;case 5125:return yC;case 36294:return xC;case 36295:return wC;case 36296:return SC;case 35678:case 36198:case 36298:case 36306:case 35682:return MC;case 35679:case 36299:case 36307:return EC;case 35680:case 36300:case 36308:case 36293:return CC;case 36289:case 36303:case 36311:case 36292:return TC}}var Hf=class{constructor(e,t,i){this.id=e,this.addr=i,this.cache=[],this.type=t.type,this.setValue=aC(t.type)}},Gf=class{constructor(e,t,i){this.id=e,this.addr=i,this.cache=[],this.type=t.type,this.size=t.size,this.setValue=bC(t.type)}},Wf=class{constructor(e){this.id=e,this.seq=[],this.map={}}setValue(e,t,i){let r=this.seq;for(let s=0,o=r.length;s!==o;++s){let a=r[s];a.setValue(e,t[a.id],i)}}},Qd=/(\w+)(\])?(\[|\.)?/g;function nv(n,e){n.seq.push(e),n.map[e.id]=e}function AC(n,e,t){let i=n.name,r=i.length;for(Qd.lastIndex=0;;){let s=Qd.exec(i),o=Qd.lastIndex,a=s[1],l=s[2]==="]",c=s[3];if(l&&(a=a|0),c===void 0||c==="["&&o+2===r){nv(t,c===void 0?new Hf(a,n,e):new Gf(a,n,e));break}else{let p=t.map[a];p===void 0&&(p=new Wf(a),nv(t,p)),t=p}}}var Os=class{constructor(e,t){this.seq=[],this.map={};let i=e.getProgramParameter(t,e.ACTIVE_UNIFORMS);for(let r=0;r<i;++r){let s=e.getActiveUniform(t,r),o=e.getUniformLocation(t,s.name);AC(s,o,this)}}setValue(e,t,i,r){let s=this.map[t];s!==void 0&&s.setValue(e,i,r)}setOptional(e,t,i){let r=t[i];r!==void 0&&this.setValue(e,i,r)}static upload(e,t,i,r){for(let s=0,o=t.length;s!==o;++s){let a=t[s],l=i[a.id];l.needsUpdate!==!1&&a.setValue(e,l.value,r)}}static seqWithValue(e,t){let i=[];for(let r=0,s=e.length;r!==s;++r){let o=e[r];o.id in t&&i.push(o)}return i}};function iv(n,e,t){let i=n.createShader(e);return n.shaderSource(i,t),n.compileShader(i),i}var RC=37297,PC=0;function IC(n,e){let t=n.split(`
`),i=[],r=Math.max(e-6,0),s=Math.min(e+6,t.length);for(let o=r;o<s;o++){let a=o+1;i.push(`${a===e?">":" "} ${a}: ${t[o]}`)}return i.join(`
`)}function LC(n){let e=Ke.getPrimaries(Ke.workingColorSpace),t=Ke.getPrimaries(n),i;switch(e===t?i="":e===uc&&t===cc?i="LinearDisplayP3ToLinearSRGB":e===cc&&t===uc&&(i="LinearSRGBToLinearDisplayP3"),n){case Ji:case Uc:return[i,"LinearTransferOETF"];case un:case xp:return[i,"sRGBTransferOETF"];default:return console.warn("THREE.WebGLProgram: Unsupported color space:",n),[i,"LinearTransferOETF"]}}function rv(n,e,t){let i=n.getShaderParameter(e,n.COMPILE_STATUS),r=n.getShaderInfoLog(e).trim();if(i&&r==="")return"";let s=/ERROR: 0:(\d+)/.exec(r);if(s){let o=parseInt(s[1]);return t.toUpperCase()+`

`+r+`

`+IC(n.getShaderSource(e),o)}else return r}function UC(n,e){let t=LC(e);return`vec4 ${n}( vec4 value ) { return ${t[0]}( ${t[1]}( value ) ); }`}function NC(n,e){let t;switch(e){case oS:t="Linear";break;case aS:t="Reinhard";break;case lS:t="OptimizedCineon";break;case cS:t="ACESFilmic";break;case hS:t="AgX";break;case dS:t="Neutral";break;case uS:t="Custom";break;default:console.warn("THREE.WebGLProgram: Unsupported toneMapping:",e),t="Linear"}return"vec3 "+n+"( vec3 color ) { return "+t+"ToneMapping( color ); }"}function DC(n){return[n.extensionClipCullDistance?"#extension GL_ANGLE_clip_cull_distance : require":"",n.extensionMultiDraw?"#extension GL_ANGLE_multi_draw : require":""].filter(Qo).join(`
`)}function OC(n){let e=[];for(let t in n){let i=n[t];i!==!1&&e.push("#define "+t+" "+i)}return e.join(`
`)}function FC(n,e){let t={},i=n.getProgramParameter(e,n.ACTIVE_ATTRIBUTES);for(let r=0;r<i;r++){let s=n.getActiveAttrib(e,r),o=s.name,a=1;s.type===n.FLOAT_MAT2&&(a=2),s.type===n.FLOAT_MAT3&&(a=3),s.type===n.FLOAT_MAT4&&(a=4),t[o]={type:s.type,location:n.getAttribLocation(e,o),locationSize:a}}return t}function Qo(n){return n!==""}function sv(n,e){let t=e.numSpotLightShadows+e.numSpotLightMaps-e.numSpotLightShadowsWithMaps;return n.replace(/NUM_DIR_LIGHTS/g,e.numDirLights).replace(/NUM_SPOT_LIGHTS/g,e.numSpotLights).replace(/NUM_SPOT_LIGHT_MAPS/g,e.numSpotLightMaps).replace(/NUM_SPOT_LIGHT_COORDS/g,t).replace(/NUM_RECT_AREA_LIGHTS/g,e.numRectAreaLights).replace(/NUM_POINT_LIGHTS/g,e.numPointLights).replace(/NUM_HEMI_LIGHTS/g,e.numHemiLights).replace(/NUM_DIR_LIGHT_SHADOWS/g,e.numDirLightShadows).replace(/NUM_SPOT_LIGHT_SHADOWS_WITH_MAPS/g,e.numSpotLightShadowsWithMaps).replace(/NUM_SPOT_LIGHT_SHADOWS/g,e.numSpotLightShadows).replace(/NUM_POINT_LIGHT_SHADOWS/g,e.numPointLightShadows)}function ov(n,e){return n.replace(/NUM_CLIPPING_PLANES/g,e.numClippingPlanes).replace(/UNION_CLIPPING_PLANES/g,e.numClippingPlanes-e.numClipIntersection)}var BC=/^[ \t]*#include +<([\w\d./]+)>/gm;function Xf(n){return n.replace(BC,zC)}var kC=new Map;function zC(n,e){let t=Le[e];if(t===void 0){let i=kC.get(e);if(i!==void 0)t=Le[i],console.warn('THREE.WebGLRenderer: Shader chunk "%s" has been deprecated. Use "%s" instead.',e,i);else throw new Error("Can not resolve #include <"+e+">")}return Xf(t)}var VC=/#pragma unroll_loop_start\s+for\s*\(\s*int\s+i\s*=\s*(\d+)\s*;\s*i\s*<\s*(\d+)\s*;\s*i\s*\+\+\s*\)\s*{([\s\S]+?)}\s+#pragma unroll_loop_end/g;function av(n){return n.replace(VC,HC)}function HC(n,e,t,i){let r="";for(let s=parseInt(e);s<parseInt(t);s++)r+=i.replace(/\[\s*i\s*\]/g,"[ "+s+" ]").replace(/UNROLLED_LOOP_INDEX/g,s);return r}function lv(n){let e=`precision ${n.precision} float;
	precision ${n.precision} int;
	precision ${n.precision} sampler2D;
	precision ${n.precision} samplerCube;
	precision ${n.precision} sampler3D;
	precision ${n.precision} sampler2DArray;
	precision ${n.precision} sampler2DShadow;
	precision ${n.precision} samplerCubeShadow;
	precision ${n.precision} sampler2DArrayShadow;
	precision ${n.precision} isampler2D;
	precision ${n.precision} isampler3D;
	precision ${n.precision} isamplerCube;
	precision ${n.precision} isampler2DArray;
	precision ${n.precision} usampler2D;
	precision ${n.precision} usampler3D;
	precision ${n.precision} usamplerCube;
	precision ${n.precision} usampler2DArray;
	`;return n.precision==="highp"?e+=`
#define HIGH_PRECISION`:n.precision==="mediump"?e+=`
#define MEDIUM_PRECISION`:n.precision==="lowp"&&(e+=`
#define LOW_PRECISION`),e}function GC(n){let e="SHADOWMAP_TYPE_BASIC";return n.shadowMapType===mv?e="SHADOWMAP_TYPE_PCF":n.shadowMapType===fp?e="SHADOWMAP_TYPE_PCF_SOFT":n.shadowMapType===ai&&(e="SHADOWMAP_TYPE_VSM"),e}function WC(n){let e="ENVMAP_TYPE_CUBE";if(n.envMap)switch(n.envMapMode){case Fs:case Bs:e="ENVMAP_TYPE_CUBE";break;case Lc:e="ENVMAP_TYPE_CUBE_UV";break}return e}function XC(n){let e="ENVMAP_MODE_REFLECTION";if(n.envMap)switch(n.envMapMode){case Bs:e="ENVMAP_MODE_REFRACTION";break}return e}function YC(n){let e="ENVMAP_BLENDING_NONE";if(n.envMap)switch(n.combine){case gv:e="ENVMAP_BLENDING_MULTIPLY";break;case rS:e="ENVMAP_BLENDING_MIX";break;case sS:e="ENVMAP_BLENDING_ADD";break}return e}function qC(n){let e=n.envMapCubeUVHeight;if(e===null)return null;let t=Math.log2(e)-2,i=1/e;return{texelWidth:1/(3*Math.max(Math.pow(2,t),7*16)),texelHeight:i,maxMip:t}}function $C(n,e,t,i){let r=n.getContext(),s=t.defines,o=t.vertexShader,a=t.fragmentShader,l=GC(t),c=WC(t),u=XC(t),p=YC(t),f=qC(t),m=DC(t),_=OC(s),y=r.createProgram(),d,h,g=t.glslVersion?"#version "+t.glslVersion+`
`:"";t.isRawShaderMaterial?(d=["#define SHADER_TYPE "+t.shaderType,"#define SHADER_NAME "+t.shaderName,_].filter(Qo).join(`
`),d.length>0&&(d+=`
`),h=["#define SHADER_TYPE "+t.shaderType,"#define SHADER_NAME "+t.shaderName,_].filter(Qo).join(`
`),h.length>0&&(h+=`
`)):(d=[lv(t),"#define SHADER_TYPE "+t.shaderType,"#define SHADER_NAME "+t.shaderName,_,t.extensionClipCullDistance?"#define USE_CLIP_DISTANCE":"",t.batching?"#define USE_BATCHING":"",t.batchingColor?"#define USE_BATCHING_COLOR":"",t.instancing?"#define USE_INSTANCING":"",t.instancingColor?"#define USE_INSTANCING_COLOR":"",t.instancingMorph?"#define USE_INSTANCING_MORPH":"",t.useFog&&t.fog?"#define USE_FOG":"",t.useFog&&t.fogExp2?"#define FOG_EXP2":"",t.map?"#define USE_MAP":"",t.envMap?"#define USE_ENVMAP":"",t.envMap?"#define "+u:"",t.lightMap?"#define USE_LIGHTMAP":"",t.aoMap?"#define USE_AOMAP":"",t.bumpMap?"#define USE_BUMPMAP":"",t.normalMap?"#define USE_NORMALMAP":"",t.normalMapObjectSpace?"#define USE_NORMALMAP_OBJECTSPACE":"",t.normalMapTangentSpace?"#define USE_NORMALMAP_TANGENTSPACE":"",t.displacementMap?"#define USE_DISPLACEMENTMAP":"",t.emissiveMap?"#define USE_EMISSIVEMAP":"",t.anisotropy?"#define USE_ANISOTROPY":"",t.anisotropyMap?"#define USE_ANISOTROPYMAP":"",t.clearcoatMap?"#define USE_CLEARCOATMAP":"",t.clearcoatRoughnessMap?"#define USE_CLEARCOAT_ROUGHNESSMAP":"",t.clearcoatNormalMap?"#define USE_CLEARCOAT_NORMALMAP":"",t.iridescenceMap?"#define USE_IRIDESCENCEMAP":"",t.iridescenceThicknessMap?"#define USE_IRIDESCENCE_THICKNESSMAP":"",t.specularMap?"#define USE_SPECULARMAP":"",t.specularColorMap?"#define USE_SPECULAR_COLORMAP":"",t.specularIntensityMap?"#define USE_SPECULAR_INTENSITYMAP":"",t.roughnessMap?"#define USE_ROUGHNESSMAP":"",t.metalnessMap?"#define USE_METALNESSMAP":"",t.alphaMap?"#define USE_ALPHAMAP":"",t.alphaHash?"#define USE_ALPHAHASH":"",t.transmission?"#define USE_TRANSMISSION":"",t.transmissionMap?"#define USE_TRANSMISSIONMAP":"",t.thicknessMap?"#define USE_THICKNESSMAP":"",t.sheenColorMap?"#define USE_SHEEN_COLORMAP":"",t.sheenRoughnessMap?"#define USE_SHEEN_ROUGHNESSMAP":"",t.mapUv?"#define MAP_UV "+t.mapUv:"",t.alphaMapUv?"#define ALPHAMAP_UV "+t.alphaMapUv:"",t.lightMapUv?"#define LIGHTMAP_UV "+t.lightMapUv:"",t.aoMapUv?"#define AOMAP_UV "+t.aoMapUv:"",t.emissiveMapUv?"#define EMISSIVEMAP_UV "+t.emissiveMapUv:"",t.bumpMapUv?"#define BUMPMAP_UV "+t.bumpMapUv:"",t.normalMapUv?"#define NORMALMAP_UV "+t.normalMapUv:"",t.displacementMapUv?"#define DISPLACEMENTMAP_UV "+t.displacementMapUv:"",t.metalnessMapUv?"#define METALNESSMAP_UV "+t.metalnessMapUv:"",t.roughnessMapUv?"#define ROUGHNESSMAP_UV "+t.roughnessMapUv:"",t.anisotropyMapUv?"#define ANISOTROPYMAP_UV "+t.anisotropyMapUv:"",t.clearcoatMapUv?"#define CLEARCOATMAP_UV "+t.clearcoatMapUv:"",t.clearcoatNormalMapUv?"#define CLEARCOAT_NORMALMAP_UV "+t.clearcoatNormalMapUv:"",t.clearcoatRoughnessMapUv?"#define CLEARCOAT_ROUGHNESSMAP_UV "+t.clearcoatRoughnessMapUv:"",t.iridescenceMapUv?"#define IRIDESCENCEMAP_UV "+t.iridescenceMapUv:"",t.iridescenceThicknessMapUv?"#define IRIDESCENCE_THICKNESSMAP_UV "+t.iridescenceThicknessMapUv:"",t.sheenColorMapUv?"#define SHEEN_COLORMAP_UV "+t.sheenColorMapUv:"",t.sheenRoughnessMapUv?"#define SHEEN_ROUGHNESSMAP_UV "+t.sheenRoughnessMapUv:"",t.specularMapUv?"#define SPECULARMAP_UV "+t.specularMapUv:"",t.specularColorMapUv?"#define SPECULAR_COLORMAP_UV "+t.specularColorMapUv:"",t.specularIntensityMapUv?"#define SPECULAR_INTENSITYMAP_UV "+t.specularIntensityMapUv:"",t.transmissionMapUv?"#define TRANSMISSIONMAP_UV "+t.transmissionMapUv:"",t.thicknessMapUv?"#define THICKNESSMAP_UV "+t.thicknessMapUv:"",t.vertexTangents&&t.flatShading===!1?"#define USE_TANGENT":"",t.vertexColors?"#define USE_COLOR":"",t.vertexAlphas?"#define USE_COLOR_ALPHA":"",t.vertexUv1s?"#define USE_UV1":"",t.vertexUv2s?"#define USE_UV2":"",t.vertexUv3s?"#define USE_UV3":"",t.pointsUvs?"#define USE_POINTS_UV":"",t.flatShading?"#define FLAT_SHADED":"",t.skinning?"#define USE_SKINNING":"",t.morphTargets?"#define USE_MORPHTARGETS":"",t.morphNormals&&t.flatShading===!1?"#define USE_MORPHNORMALS":"",t.morphColors?"#define USE_MORPHCOLORS":"",t.morphTargetsCount>0?"#define MORPHTARGETS_TEXTURE_STRIDE "+t.morphTextureStride:"",t.morphTargetsCount>0?"#define MORPHTARGETS_COUNT "+t.morphTargetsCount:"",t.doubleSided?"#define DOUBLE_SIDED":"",t.flipSided?"#define FLIP_SIDED":"",t.shadowMapEnabled?"#define USE_SHADOWMAP":"",t.shadowMapEnabled?"#define "+l:"",t.sizeAttenuation?"#define USE_SIZEATTENUATION":"",t.numLightProbes>0?"#define USE_LIGHT_PROBES":"",t.logarithmicDepthBuffer?"#define USE_LOGDEPTHBUF":"","uniform mat4 modelMatrix;","uniform mat4 modelViewMatrix;","uniform mat4 projectionMatrix;","uniform mat4 viewMatrix;","uniform mat3 normalMatrix;","uniform vec3 cameraPosition;","uniform bool isOrthographic;","#ifdef USE_INSTANCING","	attribute mat4 instanceMatrix;","#endif","#ifdef USE_INSTANCING_COLOR","	attribute vec3 instanceColor;","#endif","#ifdef USE_INSTANCING_MORPH","	uniform sampler2D morphTexture;","#endif","attribute vec3 position;","attribute vec3 normal;","attribute vec2 uv;","#ifdef USE_UV1","	attribute vec2 uv1;","#endif","#ifdef USE_UV2","	attribute vec2 uv2;","#endif","#ifdef USE_UV3","	attribute vec2 uv3;","#endif","#ifdef USE_TANGENT","	attribute vec4 tangent;","#endif","#if defined( USE_COLOR_ALPHA )","	attribute vec4 color;","#elif defined( USE_COLOR )","	attribute vec3 color;","#endif","#ifdef USE_SKINNING","	attribute vec4 skinIndex;","	attribute vec4 skinWeight;","#endif",`
`].filter(Qo).join(`
`),h=[lv(t),"#define SHADER_TYPE "+t.shaderType,"#define SHADER_NAME "+t.shaderName,_,t.useFog&&t.fog?"#define USE_FOG":"",t.useFog&&t.fogExp2?"#define FOG_EXP2":"",t.alphaToCoverage?"#define ALPHA_TO_COVERAGE":"",t.map?"#define USE_MAP":"",t.matcap?"#define USE_MATCAP":"",t.envMap?"#define USE_ENVMAP":"",t.envMap?"#define "+c:"",t.envMap?"#define "+u:"",t.envMap?"#define "+p:"",f?"#define CUBEUV_TEXEL_WIDTH "+f.texelWidth:"",f?"#define CUBEUV_TEXEL_HEIGHT "+f.texelHeight:"",f?"#define CUBEUV_MAX_MIP "+f.maxMip+".0":"",t.lightMap?"#define USE_LIGHTMAP":"",t.aoMap?"#define USE_AOMAP":"",t.bumpMap?"#define USE_BUMPMAP":"",t.normalMap?"#define USE_NORMALMAP":"",t.normalMapObjectSpace?"#define USE_NORMALMAP_OBJECTSPACE":"",t.normalMapTangentSpace?"#define USE_NORMALMAP_TANGENTSPACE":"",t.emissiveMap?"#define USE_EMISSIVEMAP":"",t.anisotropy?"#define USE_ANISOTROPY":"",t.anisotropyMap?"#define USE_ANISOTROPYMAP":"",t.clearcoat?"#define USE_CLEARCOAT":"",t.clearcoatMap?"#define USE_CLEARCOATMAP":"",t.clearcoatRoughnessMap?"#define USE_CLEARCOAT_ROUGHNESSMAP":"",t.clearcoatNormalMap?"#define USE_CLEARCOAT_NORMALMAP":"",t.dispersion?"#define USE_DISPERSION":"",t.iridescence?"#define USE_IRIDESCENCE":"",t.iridescenceMap?"#define USE_IRIDESCENCEMAP":"",t.iridescenceThicknessMap?"#define USE_IRIDESCENCE_THICKNESSMAP":"",t.specularMap?"#define USE_SPECULARMAP":"",t.specularColorMap?"#define USE_SPECULAR_COLORMAP":"",t.specularIntensityMap?"#define USE_SPECULAR_INTENSITYMAP":"",t.roughnessMap?"#define USE_ROUGHNESSMAP":"",t.metalnessMap?"#define USE_METALNESSMAP":"",t.alphaMap?"#define USE_ALPHAMAP":"",t.alphaTest?"#define USE_ALPHATEST":"",t.alphaHash?"#define USE_ALPHAHASH":"",t.sheen?"#define USE_SHEEN":"",t.sheenColorMap?"#define USE_SHEEN_COLORMAP":"",t.sheenRoughnessMap?"#define USE_SHEEN_ROUGHNESSMAP":"",t.transmission?"#define USE_TRANSMISSION":"",t.transmissionMap?"#define USE_TRANSMISSIONMAP":"",t.thicknessMap?"#define USE_THICKNESSMAP":"",t.vertexTangents&&t.flatShading===!1?"#define USE_TANGENT":"",t.vertexColors||t.instancingColor||t.batchingColor?"#define USE_COLOR":"",t.vertexAlphas?"#define USE_COLOR_ALPHA":"",t.vertexUv1s?"#define USE_UV1":"",t.vertexUv2s?"#define USE_UV2":"",t.vertexUv3s?"#define USE_UV3":"",t.pointsUvs?"#define USE_POINTS_UV":"",t.gradientMap?"#define USE_GRADIENTMAP":"",t.flatShading?"#define FLAT_SHADED":"",t.doubleSided?"#define DOUBLE_SIDED":"",t.flipSided?"#define FLIP_SIDED":"",t.shadowMapEnabled?"#define USE_SHADOWMAP":"",t.shadowMapEnabled?"#define "+l:"",t.premultipliedAlpha?"#define PREMULTIPLIED_ALPHA":"",t.numLightProbes>0?"#define USE_LIGHT_PROBES":"",t.decodeVideoTexture?"#define DECODE_VIDEO_TEXTURE":"",t.logarithmicDepthBuffer?"#define USE_LOGDEPTHBUF":"","uniform mat4 viewMatrix;","uniform vec3 cameraPosition;","uniform bool isOrthographic;",t.toneMapping!==qi?"#define TONE_MAPPING":"",t.toneMapping!==qi?Le.tonemapping_pars_fragment:"",t.toneMapping!==qi?NC("toneMapping",t.toneMapping):"",t.dithering?"#define DITHERING":"",t.opaque?"#define OPAQUE":"",Le.colorspace_pars_fragment,UC("linearToOutputTexel",t.outputColorSpace),t.useDepthPacking?"#define DEPTH_PACKING "+t.depthPacking:"",`
`].filter(Qo).join(`
`)),o=Xf(o),o=sv(o,t),o=ov(o,t),a=Xf(a),a=sv(a,t),a=ov(a,t),o=av(o),a=av(a),t.isRawShaderMaterial!==!0&&(g=`#version 300 es
`,d=[m,"#define attribute in","#define varying out","#define texture2D texture"].join(`
`)+`
`+d,h=["#define varying in",t.glslVersion===E_?"":"layout(location = 0) out highp vec4 pc_fragColor;",t.glslVersion===E_?"":"#define gl_FragColor pc_fragColor","#define gl_FragDepthEXT gl_FragDepth","#define texture2D texture","#define textureCube texture","#define texture2DProj textureProj","#define texture2DLodEXT textureLod","#define texture2DProjLodEXT textureProjLod","#define textureCubeLodEXT textureLod","#define texture2DGradEXT textureGrad","#define texture2DProjGradEXT textureProjGrad","#define textureCubeGradEXT textureGrad"].join(`
`)+`
`+h);let v=g+d+o,x=g+h+a,R=iv(r,r.VERTEX_SHADER,v),C=iv(r,r.FRAGMENT_SHADER,x);r.attachShader(y,R),r.attachShader(y,C),t.index0AttributeName!==void 0?r.bindAttribLocation(y,0,t.index0AttributeName):t.morphTargets===!0&&r.bindAttribLocation(y,0,"position"),r.linkProgram(y);function T(P){if(n.debug.checkShaderErrors){let W=r.getProgramInfoLog(y).trim(),k=r.getShaderInfoLog(R).trim(),G=r.getShaderInfoLog(C).trim(),$=!0,V=!0;if(r.getProgramParameter(y,r.LINK_STATUS)===!1)if($=!1,typeof n.debug.onShaderError=="function")n.debug.onShaderError(r,y,R,C);else{let K=rv(r,R,"vertex"),z=rv(r,C,"fragment");console.error("THREE.WebGLProgram: Shader Error "+r.getError()+" - VALIDATE_STATUS "+r.getProgramParameter(y,r.VALIDATE_STATUS)+`

Material Name: `+P.name+`
Material Type: `+P.type+`

Program Info Log: `+W+`
`+K+`
`+z)}else W!==""?console.warn("THREE.WebGLProgram: Program Info Log:",W):(k===""||G==="")&&(V=!1);V&&(P.diagnostics={runnable:$,programLog:W,vertexShader:{log:k,prefix:d},fragmentShader:{log:G,prefix:h}})}r.deleteShader(R),r.deleteShader(C),L=new Os(r,y),E=FC(r,y)}let L;this.getUniforms=function(){return L===void 0&&T(this),L};let E;this.getAttributes=function(){return E===void 0&&T(this),E};let M=t.rendererExtensionParallelShaderCompile===!1;return this.isReady=function(){return M===!1&&(M=r.getProgramParameter(y,RC)),M},this.destroy=function(){i.releaseStatesOfProgram(this),r.deleteProgram(y),this.program=void 0},this.type=t.shaderType,this.name=t.shaderName,this.id=PC++,this.cacheKey=e,this.usedTimes=1,this.program=y,this.vertexShader=R,this.fragmentShader=C,this}var ZC=0,Yf=class{constructor(){this.shaderCache=new Map,this.materialCache=new Map}update(e){let t=e.vertexShader,i=e.fragmentShader,r=this._getShaderStage(t),s=this._getShaderStage(i),o=this._getShaderCacheForMaterial(e);return o.has(r)===!1&&(o.add(r),r.usedTimes++),o.has(s)===!1&&(o.add(s),s.usedTimes++),this}remove(e){let t=this.materialCache.get(e);for(let i of t)i.usedTimes--,i.usedTimes===0&&this.shaderCache.delete(i.code);return this.materialCache.delete(e),this}getVertexShaderID(e){return this._getShaderStage(e.vertexShader).id}getFragmentShaderID(e){return this._getShaderStage(e.fragmentShader).id}dispose(){this.shaderCache.clear(),this.materialCache.clear()}_getShaderCacheForMaterial(e){let t=this.materialCache,i=t.get(e);return i===void 0&&(i=new Set,t.set(e,i)),i}_getShaderStage(e){let t=this.shaderCache,i=t.get(e);return i===void 0&&(i=new qf(e),t.set(e,i)),i}},qf=class{constructor(e){this.id=ZC++,this.code=e,this.usedTimes=0}};function JC(n,e,t,i,r,s,o){let a=new pc,l=new Yf,c=new Set,u=[],p=r.logarithmicDepthBuffer,f=r.vertexTextures,m=r.precision,_={MeshDepthMaterial:"depth",MeshDistanceMaterial:"distanceRGBA",MeshNormalMaterial:"normal",MeshBasicMaterial:"basic",MeshLambertMaterial:"lambert",MeshPhongMaterial:"phong",MeshToonMaterial:"toon",MeshStandardMaterial:"physical",MeshPhysicalMaterial:"physical",MeshMatcapMaterial:"matcap",LineBasicMaterial:"basic",LineDashedMaterial:"dashed",PointsMaterial:"points",ShadowMaterial:"shadow",SpriteMaterial:"sprite"};function y(E){return c.add(E),E===0?"uv":`uv${E}`}function d(E,M,P,W,k){let G=W.fog,$=k.geometry,V=E.isMeshStandardMaterial?W.environment:null,K=(E.isMeshStandardMaterial?t:e).get(E.envMap||V),z=K&&K.mapping===Lc?K.image.height:null,he=_[E.type];E.precision!==null&&(m=r.getMaxPrecision(E.precision),m!==E.precision&&console.warn("THREE.WebGLProgram.getParameters:",E.precision,"not supported, using",m,"instead."));let ge=$.morphAttributes.position||$.morphAttributes.normal||$.morphAttributes.color,_e=ge!==void 0?ge.length:0,Ve=0;$.morphAttributes.position!==void 0&&(Ve=1),$.morphAttributes.normal!==void 0&&(Ve=2),$.morphAttributes.color!==void 0&&(Ve=3);let je,H,Q,fe;if(he){let Xe=Wn[he];je=Xe.vertexShader,H=Xe.fragmentShader}else je=E.vertexShader,H=E.fragmentShader,l.update(E),Q=l.getVertexShaderID(E),fe=l.getFragmentShaderID(E);let ce=n.getRenderTarget(),Re=k.isInstancedMesh===!0,Ne=k.isBatchedMesh===!0,Be=!!E.map,lt=!!E.matcap,A=!!K,pt=!!E.aoMap,Je=!!E.lightMap,Qe=!!E.bumpMap,ye=!!E.normalMap,mt=!!E.displacementMap,be=!!E.emissiveMap,Pe=!!E.metalnessMap,b=!!E.roughnessMap,w=E.anisotropy>0,B=E.clearcoat>0,Z=E.dispersion>0,J=E.iridescence>0,q=E.sheen>0,xe=E.transmission>0,se=w&&!!E.anisotropyMap,ue=B&&!!E.clearcoatMap,Ie=B&&!!E.clearcoatNormalMap,ee=B&&!!E.clearcoatRoughnessMap,ae=J&&!!E.iridescenceMap,He=J&&!!E.iridescenceThicknessMap,Te=q&&!!E.sheenColorMap,de=q&&!!E.sheenRoughnessMap,Ae=!!E.specularMap,De=!!E.specularColorMap,at=!!E.specularIntensityMap,I=xe&&!!E.transmissionMap,te=xe&&!!E.thicknessMap,X=!!E.gradientMap,Y=!!E.alphaMap,ie=E.alphaTest>0,Se=!!E.alphaHash,Ge=!!E.extensions,gt=qi;E.toneMapped&&(ce===null||ce.isXRRenderTarget===!0)&&(gt=n.toneMapping);let bt={shaderID:he,shaderType:E.type,shaderName:E.name,vertexShader:je,fragmentShader:H,defines:E.defines,customVertexShaderID:Q,customFragmentShaderID:fe,isRawShaderMaterial:E.isRawShaderMaterial===!0,glslVersion:E.glslVersion,precision:m,batching:Ne,batchingColor:Ne&&k._colorsTexture!==null,instancing:Re,instancingColor:Re&&k.instanceColor!==null,instancingMorph:Re&&k.morphTexture!==null,supportsVertexTextures:f,outputColorSpace:ce===null?n.outputColorSpace:ce.isXRRenderTarget===!0?ce.texture.colorSpace:Ji,alphaToCoverage:!!E.alphaToCoverage,map:Be,matcap:lt,envMap:A,envMapMode:A&&K.mapping,envMapCubeUVHeight:z,aoMap:pt,lightMap:Je,bumpMap:Qe,normalMap:ye,displacementMap:f&&mt,emissiveMap:be,normalMapObjectSpace:ye&&E.normalMapType===gS,normalMapTangentSpace:ye&&E.normalMapType===Av,metalnessMap:Pe,roughnessMap:b,anisotropy:w,anisotropyMap:se,clearcoat:B,clearcoatMap:ue,clearcoatNormalMap:Ie,clearcoatRoughnessMap:ee,dispersion:Z,iridescence:J,iridescenceMap:ae,iridescenceThicknessMap:He,sheen:q,sheenColorMap:Te,sheenRoughnessMap:de,specularMap:Ae,specularColorMap:De,specularIntensityMap:at,transmission:xe,transmissionMap:I,thicknessMap:te,gradientMap:X,opaque:E.transparent===!1&&E.blending===Us&&E.alphaToCoverage===!1,alphaMap:Y,alphaTest:ie,alphaHash:Se,combine:E.combine,mapUv:Be&&y(E.map.channel),aoMapUv:pt&&y(E.aoMap.channel),lightMapUv:Je&&y(E.lightMap.channel),bumpMapUv:Qe&&y(E.bumpMap.channel),normalMapUv:ye&&y(E.normalMap.channel),displacementMapUv:mt&&y(E.displacementMap.channel),emissiveMapUv:be&&y(E.emissiveMap.channel),metalnessMapUv:Pe&&y(E.metalnessMap.channel),roughnessMapUv:b&&y(E.roughnessMap.channel),anisotropyMapUv:se&&y(E.anisotropyMap.channel),clearcoatMapUv:ue&&y(E.clearcoatMap.channel),clearcoatNormalMapUv:Ie&&y(E.clearcoatNormalMap.channel),clearcoatRoughnessMapUv:ee&&y(E.clearcoatRoughnessMap.channel),iridescenceMapUv:ae&&y(E.iridescenceMap.channel),iridescenceThicknessMapUv:He&&y(E.iridescenceThicknessMap.channel),sheenColorMapUv:Te&&y(E.sheenColorMap.channel),sheenRoughnessMapUv:de&&y(E.sheenRoughnessMap.channel),specularMapUv:Ae&&y(E.specularMap.channel),specularColorMapUv:De&&y(E.specularColorMap.channel),specularIntensityMapUv:at&&y(E.specularIntensityMap.channel),transmissionMapUv:I&&y(E.transmissionMap.channel),thicknessMapUv:te&&y(E.thicknessMap.channel),alphaMapUv:Y&&y(E.alphaMap.channel),vertexTangents:!!$.attributes.tangent&&(ye||w),vertexColors:E.vertexColors,vertexAlphas:E.vertexColors===!0&&!!$.attributes.color&&$.attributes.color.itemSize===4,pointsUvs:k.isPoints===!0&&!!$.attributes.uv&&(Be||Y),fog:!!G,useFog:E.fog===!0,fogExp2:!!G&&G.isFogExp2,flatShading:E.flatShading===!0,sizeAttenuation:E.sizeAttenuation===!0,logarithmicDepthBuffer:p,skinning:k.isSkinnedMesh===!0,morphTargets:$.morphAttributes.position!==void 0,morphNormals:$.morphAttributes.normal!==void 0,morphColors:$.morphAttributes.color!==void 0,morphTargetsCount:_e,morphTextureStride:Ve,numDirLights:M.directional.length,numPointLights:M.point.length,numSpotLights:M.spot.length,numSpotLightMaps:M.spotLightMap.length,numRectAreaLights:M.rectArea.length,numHemiLights:M.hemi.length,numDirLightShadows:M.directionalShadowMap.length,numPointLightShadows:M.pointShadowMap.length,numSpotLightShadows:M.spotShadowMap.length,numSpotLightShadowsWithMaps:M.numSpotLightShadowsWithMaps,numLightProbes:M.numLightProbes,numClippingPlanes:o.numPlanes,numClipIntersection:o.numIntersection,dithering:E.dithering,shadowMapEnabled:n.shadowMap.enabled&&P.length>0,shadowMapType:n.shadowMap.type,toneMapping:gt,decodeVideoTexture:Be&&E.map.isVideoTexture===!0&&Ke.getTransfer(E.map.colorSpace)===rt,premultipliedAlpha:E.premultipliedAlpha,doubleSided:E.side===li,flipSided:E.side===tn,useDepthPacking:E.depthPacking>=0,depthPacking:E.depthPacking||0,index0AttributeName:E.index0AttributeName,extensionClipCullDistance:Ge&&E.extensions.clipCullDistance===!0&&i.has("WEBGL_clip_cull_distance"),extensionMultiDraw:(Ge&&E.extensions.multiDraw===!0||Ne)&&i.has("WEBGL_multi_draw"),rendererExtensionParallelShaderCompile:i.has("KHR_parallel_shader_compile"),customProgramCacheKey:E.customProgramCacheKey()};return bt.vertexUv1s=c.has(1),bt.vertexUv2s=c.has(2),bt.vertexUv3s=c.has(3),c.clear(),bt}function h(E){let M=[];if(E.shaderID?M.push(E.shaderID):(M.push(E.customVertexShaderID),M.push(E.customFragmentShaderID)),E.defines!==void 0)for(let P in E.defines)M.push(P),M.push(E.defines[P]);return E.isRawShaderMaterial===!1&&(g(M,E),v(M,E),M.push(n.outputColorSpace)),M.push(E.customProgramCacheKey),M.join()}function g(E,M){E.push(M.precision),E.push(M.outputColorSpace),E.push(M.envMapMode),E.push(M.envMapCubeUVHeight),E.push(M.mapUv),E.push(M.alphaMapUv),E.push(M.lightMapUv),E.push(M.aoMapUv),E.push(M.bumpMapUv),E.push(M.normalMapUv),E.push(M.displacementMapUv),E.push(M.emissiveMapUv),E.push(M.metalnessMapUv),E.push(M.roughnessMapUv),E.push(M.anisotropyMapUv),E.push(M.clearcoatMapUv),E.push(M.clearcoatNormalMapUv),E.push(M.clearcoatRoughnessMapUv),E.push(M.iridescenceMapUv),E.push(M.iridescenceThicknessMapUv),E.push(M.sheenColorMapUv),E.push(M.sheenRoughnessMapUv),E.push(M.specularMapUv),E.push(M.specularColorMapUv),E.push(M.specularIntensityMapUv),E.push(M.transmissionMapUv),E.push(M.thicknessMapUv),E.push(M.combine),E.push(M.fogExp2),E.push(M.sizeAttenuation),E.push(M.morphTargetsCount),E.push(M.morphAttributeCount),E.push(M.numDirLights),E.push(M.numPointLights),E.push(M.numSpotLights),E.push(M.numSpotLightMaps),E.push(M.numHemiLights),E.push(M.numRectAreaLights),E.push(M.numDirLightShadows),E.push(M.numPointLightShadows),E.push(M.numSpotLightShadows),E.push(M.numSpotLightShadowsWithMaps),E.push(M.numLightProbes),E.push(M.shadowMapType),E.push(M.toneMapping),E.push(M.numClippingPlanes),E.push(M.numClipIntersection),E.push(M.depthPacking)}function v(E,M){a.disableAll(),M.supportsVertexTextures&&a.enable(0),M.instancing&&a.enable(1),M.instancingColor&&a.enable(2),M.instancingMorph&&a.enable(3),M.matcap&&a.enable(4),M.envMap&&a.enable(5),M.normalMapObjectSpace&&a.enable(6),M.normalMapTangentSpace&&a.enable(7),M.clearcoat&&a.enable(8),M.iridescence&&a.enable(9),M.alphaTest&&a.enable(10),M.vertexColors&&a.enable(11),M.vertexAlphas&&a.enable(12),M.vertexUv1s&&a.enable(13),M.vertexUv2s&&a.enable(14),M.vertexUv3s&&a.enable(15),M.vertexTangents&&a.enable(16),M.anisotropy&&a.enable(17),M.alphaHash&&a.enable(18),M.batching&&a.enable(19),M.dispersion&&a.enable(20),M.batchingColor&&a.enable(21),E.push(a.mask),a.disableAll(),M.fog&&a.enable(0),M.useFog&&a.enable(1),M.flatShading&&a.enable(2),M.logarithmicDepthBuffer&&a.enable(3),M.skinning&&a.enable(4),M.morphTargets&&a.enable(5),M.morphNormals&&a.enable(6),M.morphColors&&a.enable(7),M.premultipliedAlpha&&a.enable(8),M.shadowMapEnabled&&a.enable(9),M.doubleSided&&a.enable(10),M.flipSided&&a.enable(11),M.useDepthPacking&&a.enable(12),M.dithering&&a.enable(13),M.transmission&&a.enable(14),M.sheen&&a.enable(15),M.opaque&&a.enable(16),M.pointsUvs&&a.enable(17),M.decodeVideoTexture&&a.enable(18),M.alphaToCoverage&&a.enable(19),E.push(a.mask)}function x(E){let M=_[E.type],P;if(M){let W=Wn[M];P=VS.clone(W.uniforms)}else P=E.uniforms;return P}function R(E,M){let P;for(let W=0,k=u.length;W<k;W++){let G=u[W];if(G.cacheKey===M){P=G,++P.usedTimes;break}}return P===void 0&&(P=new $C(n,M,E,s),u.push(P)),P}function C(E){if(--E.usedTimes===0){let M=u.indexOf(E);u[M]=u[u.length-1],u.pop(),E.destroy()}}function T(E){l.remove(E)}function L(){l.dispose()}return{getParameters:d,getProgramCacheKey:h,getUniforms:x,acquireProgram:R,releaseProgram:C,releaseShaderCache:T,programs:u,dispose:L}}function KC(){let n=new WeakMap;function e(s){let o=n.get(s);return o===void 0&&(o={},n.set(s,o)),o}function t(s){n.delete(s)}function i(s,o,a){n.get(s)[o]=a}function r(){n=new WeakMap}return{get:e,remove:t,update:i,dispose:r}}function jC(n,e){return n.groupOrder!==e.groupOrder?n.groupOrder-e.groupOrder:n.renderOrder!==e.renderOrder?n.renderOrder-e.renderOrder:n.material.id!==e.material.id?n.material.id-e.material.id:n.z!==e.z?n.z-e.z:n.id-e.id}function cv(n,e){return n.groupOrder!==e.groupOrder?n.groupOrder-e.groupOrder:n.renderOrder!==e.renderOrder?n.renderOrder-e.renderOrder:n.z!==e.z?e.z-n.z:n.id-e.id}function uv(){let n=[],e=0,t=[],i=[],r=[];function s(){e=0,t.length=0,i.length=0,r.length=0}function o(p,f,m,_,y,d){let h=n[e];return h===void 0?(h={id:p.id,object:p,geometry:f,material:m,groupOrder:_,renderOrder:p.renderOrder,z:y,group:d},n[e]=h):(h.id=p.id,h.object=p,h.geometry=f,h.material=m,h.groupOrder=_,h.renderOrder=p.renderOrder,h.z=y,h.group=d),e++,h}function a(p,f,m,_,y,d){let h=o(p,f,m,_,y,d);m.transmission>0?i.push(h):m.transparent===!0?r.push(h):t.push(h)}function l(p,f,m,_,y,d){let h=o(p,f,m,_,y,d);m.transmission>0?i.unshift(h):m.transparent===!0?r.unshift(h):t.unshift(h)}function c(p,f){t.length>1&&t.sort(p||jC),i.length>1&&i.sort(f||cv),r.length>1&&r.sort(f||cv)}function u(){for(let p=e,f=n.length;p<f;p++){let m=n[p];if(m.id===null)break;m.id=null,m.object=null,m.geometry=null,m.material=null,m.group=null}}return{opaque:t,transmissive:i,transparent:r,init:s,push:a,unshift:l,finish:u,sort:c}}function QC(){let n=new WeakMap;function e(i,r){let s=n.get(i),o;return s===void 0?(o=new uv,n.set(i,[o])):r>=s.length?(o=new uv,s.push(o)):o=s[r],o}function t(){n=new WeakMap}return{get:e,dispose:t}}function eT(){let n={};return{get:function(e){if(n[e.id]!==void 0)return n[e.id];let t;switch(e.type){case"DirectionalLight":t={direction:new D,color:new Oe};break;case"SpotLight":t={position:new D,direction:new D,color:new Oe,distance:0,coneCos:0,penumbraCos:0,decay:0};break;case"PointLight":t={position:new D,color:new Oe,distance:0,decay:0};break;case"HemisphereLight":t={direction:new D,skyColor:new Oe,groundColor:new Oe};break;case"RectAreaLight":t={color:new Oe,position:new D,halfWidth:new D,halfHeight:new D};break}return n[e.id]=t,t}}}function tT(){let n={};return{get:function(e){if(n[e.id]!==void 0)return n[e.id];let t;switch(e.type){case"DirectionalLight":t={shadowIntensity:1,shadowBias:0,shadowNormalBias:0,shadowRadius:1,shadowMapSize:new ze};break;case"SpotLight":t={shadowIntensity:1,shadowBias:0,shadowNormalBias:0,shadowRadius:1,shadowMapSize:new ze};break;case"PointLight":t={shadowIntensity:1,shadowBias:0,shadowNormalBias:0,shadowRadius:1,shadowMapSize:new ze,shadowCameraNear:1,shadowCameraFar:1e3};break}return n[e.id]=t,t}}}var nT=0;function iT(n,e){return(e.castShadow?2:0)-(n.castShadow?2:0)+(e.map?1:0)-(n.map?1:0)}function rT(n){let e=new eT,t=tT(),i={version:0,hash:{directionalLength:-1,pointLength:-1,spotLength:-1,rectAreaLength:-1,hemiLength:-1,numDirectionalShadows:-1,numPointShadows:-1,numSpotShadows:-1,numSpotMaps:-1,numLightProbes:-1},ambient:[0,0,0],probe:[],directional:[],directionalShadow:[],directionalShadowMap:[],directionalShadowMatrix:[],spot:[],spotLightMap:[],spotShadow:[],spotShadowMap:[],spotLightMatrix:[],rectArea:[],rectAreaLTC1:null,rectAreaLTC2:null,point:[],pointShadow:[],pointShadowMap:[],pointShadowMatrix:[],hemi:[],numSpotLightShadowsWithMaps:0,numLightProbes:0};for(let c=0;c<9;c++)i.probe.push(new D);let r=new D,s=new et,o=new et;function a(c){let u=0,p=0,f=0;for(let E=0;E<9;E++)i.probe[E].set(0,0,0);let m=0,_=0,y=0,d=0,h=0,g=0,v=0,x=0,R=0,C=0,T=0;c.sort(iT);for(let E=0,M=c.length;E<M;E++){let P=c[E],W=P.color,k=P.intensity,G=P.distance,$=P.shadow&&P.shadow.map?P.shadow.map.texture:null;if(P.isAmbientLight)u+=W.r*k,p+=W.g*k,f+=W.b*k;else if(P.isLightProbe){for(let V=0;V<9;V++)i.probe[V].addScaledVector(P.sh.coefficients[V],k);T++}else if(P.isDirectionalLight){let V=e.get(P);if(V.color.copy(P.color).multiplyScalar(P.intensity),P.castShadow){let K=P.shadow,z=t.get(P);z.shadowIntensity=K.intensity,z.shadowBias=K.bias,z.shadowNormalBias=K.normalBias,z.shadowRadius=K.radius,z.shadowMapSize=K.mapSize,i.directionalShadow[m]=z,i.directionalShadowMap[m]=$,i.directionalShadowMatrix[m]=P.shadow.matrix,g++}i.directional[m]=V,m++}else if(P.isSpotLight){let V=e.get(P);V.position.setFromMatrixPosition(P.matrixWorld),V.color.copy(W).multiplyScalar(k),V.distance=G,V.coneCos=Math.cos(P.angle),V.penumbraCos=Math.cos(P.angle*(1-P.penumbra)),V.decay=P.decay,i.spot[y]=V;let K=P.shadow;if(P.map&&(i.spotLightMap[R]=P.map,R++,K.updateMatrices(P),P.castShadow&&C++),i.spotLightMatrix[y]=K.matrix,P.castShadow){let z=t.get(P);z.shadowIntensity=K.intensity,z.shadowBias=K.bias,z.shadowNormalBias=K.normalBias,z.shadowRadius=K.radius,z.shadowMapSize=K.mapSize,i.spotShadow[y]=z,i.spotShadowMap[y]=$,x++}y++}else if(P.isRectAreaLight){let V=e.get(P);V.color.copy(W).multiplyScalar(k),V.halfWidth.set(P.width*.5,0,0),V.halfHeight.set(0,P.height*.5,0),i.rectArea[d]=V,d++}else if(P.isPointLight){let V=e.get(P);if(V.color.copy(P.color).multiplyScalar(P.intensity),V.distance=P.distance,V.decay=P.decay,P.castShadow){let K=P.shadow,z=t.get(P);z.shadowIntensity=K.intensity,z.shadowBias=K.bias,z.shadowNormalBias=K.normalBias,z.shadowRadius=K.radius,z.shadowMapSize=K.mapSize,z.shadowCameraNear=K.camera.near,z.shadowCameraFar=K.camera.far,i.pointShadow[_]=z,i.pointShadowMap[_]=$,i.pointShadowMatrix[_]=P.shadow.matrix,v++}i.point[_]=V,_++}else if(P.isHemisphereLight){let V=e.get(P);V.skyColor.copy(P.color).multiplyScalar(k),V.groundColor.copy(P.groundColor).multiplyScalar(k),i.hemi[h]=V,h++}}d>0&&(n.has("OES_texture_float_linear")===!0?(i.rectAreaLTC1=re.LTC_FLOAT_1,i.rectAreaLTC2=re.LTC_FLOAT_2):(i.rectAreaLTC1=re.LTC_HALF_1,i.rectAreaLTC2=re.LTC_HALF_2)),i.ambient[0]=u,i.ambient[1]=p,i.ambient[2]=f;let L=i.hash;(L.directionalLength!==m||L.pointLength!==_||L.spotLength!==y||L.rectAreaLength!==d||L.hemiLength!==h||L.numDirectionalShadows!==g||L.numPointShadows!==v||L.numSpotShadows!==x||L.numSpotMaps!==R||L.numLightProbes!==T)&&(i.directional.length=m,i.spot.length=y,i.rectArea.length=d,i.point.length=_,i.hemi.length=h,i.directionalShadow.length=g,i.directionalShadowMap.length=g,i.pointShadow.length=v,i.pointShadowMap.length=v,i.spotShadow.length=x,i.spotShadowMap.length=x,i.directionalShadowMatrix.length=g,i.pointShadowMatrix.length=v,i.spotLightMatrix.length=x+R-C,i.spotLightMap.length=R,i.numSpotLightShadowsWithMaps=C,i.numLightProbes=T,L.directionalLength=m,L.pointLength=_,L.spotLength=y,L.rectAreaLength=d,L.hemiLength=h,L.numDirectionalShadows=g,L.numPointShadows=v,L.numSpotShadows=x,L.numSpotMaps=R,L.numLightProbes=T,i.version=nT++)}function l(c,u){let p=0,f=0,m=0,_=0,y=0,d=u.matrixWorldInverse;for(let h=0,g=c.length;h<g;h++){let v=c[h];if(v.isDirectionalLight){let x=i.directional[p];x.direction.setFromMatrixPosition(v.matrixWorld),r.setFromMatrixPosition(v.target.matrixWorld),x.direction.sub(r),x.direction.transformDirection(d),p++}else if(v.isSpotLight){let x=i.spot[m];x.position.setFromMatrixPosition(v.matrixWorld),x.position.applyMatrix4(d),x.direction.setFromMatrixPosition(v.matrixWorld),r.setFromMatrixPosition(v.target.matrixWorld),x.direction.sub(r),x.direction.transformDirection(d),m++}else if(v.isRectAreaLight){let x=i.rectArea[_];x.position.setFromMatrixPosition(v.matrixWorld),x.position.applyMatrix4(d),o.identity(),s.copy(v.matrixWorld),s.premultiply(d),o.extractRotation(s),x.halfWidth.set(v.width*.5,0,0),x.halfHeight.set(0,v.height*.5,0),x.halfWidth.applyMatrix4(o),x.halfHeight.applyMatrix4(o),_++}else if(v.isPointLight){let x=i.point[f];x.position.setFromMatrixPosition(v.matrixWorld),x.position.applyMatrix4(d),f++}else if(v.isHemisphereLight){let x=i.hemi[y];x.direction.setFromMatrixPosition(v.matrixWorld),x.direction.transformDirection(d),y++}}}return{setup:a,setupView:l,state:i}}function hv(n){let e=new rT(n),t=[],i=[];function r(u){c.camera=u,t.length=0,i.length=0}function s(u){t.push(u)}function o(u){i.push(u)}function a(){e.setup(t)}function l(u){e.setupView(t,u)}let c={lightsArray:t,shadowsArray:i,camera:null,lights:e,transmissionRenderTarget:{}};return{init:r,state:c,setupLights:a,setupLightsView:l,pushLight:s,pushShadow:o}}function sT(n){let e=new WeakMap;function t(r,s=0){let o=e.get(r),a;return o===void 0?(a=new hv(n),e.set(r,[a])):s>=o.length?(a=new hv(n),o.push(a)):a=o[s],a}function i(){e=new WeakMap}return{get:t,dispose:i}}var $f=class extends Ar{constructor(e){super(),this.isMeshDepthMaterial=!0,this.type="MeshDepthMaterial",this.depthPacking=pS,this.map=null,this.alphaMap=null,this.displacementMap=null,this.displacementScale=1,this.displacementBias=0,this.wireframe=!1,this.wireframeLinewidth=1,this.setValues(e)}copy(e){return super.copy(e),this.depthPacking=e.depthPacking,this.map=e.map,this.alphaMap=e.alphaMap,this.displacementMap=e.displacementMap,this.displacementScale=e.displacementScale,this.displacementBias=e.displacementBias,this.wireframe=e.wireframe,this.wireframeLinewidth=e.wireframeLinewidth,this}},Zf=class extends Ar{constructor(e){super(),this.isMeshDistanceMaterial=!0,this.type="MeshDistanceMaterial",this.map=null,this.alphaMap=null,this.displacementMap=null,this.displacementScale=1,this.displacementBias=0,this.setValues(e)}copy(e){return super.copy(e),this.map=e.map,this.alphaMap=e.alphaMap,this.displacementMap=e.displacementMap,this.displacementScale=e.displacementScale,this.displacementBias=e.displacementBias,this}},oT=`void main() {
	gl_Position = vec4( position, 1.0 );
}`,aT=`uniform sampler2D shadow_pass;
uniform vec2 resolution;
uniform float radius;
#include <packing>
void main() {
	const float samples = float( VSM_SAMPLES );
	float mean = 0.0;
	float squared_mean = 0.0;
	float uvStride = samples <= 1.0 ? 0.0 : 2.0 / ( samples - 1.0 );
	float uvStart = samples <= 1.0 ? 0.0 : - 1.0;
	for ( float i = 0.0; i < samples; i ++ ) {
		float uvOffset = uvStart + i * uvStride;
		#ifdef HORIZONTAL_PASS
			vec2 distribution = unpackRGBATo2Half( texture2D( shadow_pass, ( gl_FragCoord.xy + vec2( uvOffset, 0.0 ) * radius ) / resolution ) );
			mean += distribution.x;
			squared_mean += distribution.y * distribution.y + distribution.x * distribution.x;
		#else
			float depth = unpackRGBAToDepth( texture2D( shadow_pass, ( gl_FragCoord.xy + vec2( 0.0, uvOffset ) * radius ) / resolution ) );
			mean += depth;
			squared_mean += depth * depth;
		#endif
	}
	mean = mean / samples;
	squared_mean = squared_mean / samples;
	float std_dev = sqrt( squared_mean - mean * mean );
	gl_FragColor = pack2HalfToRGBA( vec2( mean, std_dev ) );
}`;function lT(n,e,t){let i=new ra,r=new ze,s=new ze,o=new wt,a=new $f({depthPacking:mS}),l=new Zf,c={},u=t.maxTextureSize,p={[$i]:tn,[tn]:$i,[li]:li},f=new Mn({defines:{VSM_SAMPLES:8},uniforms:{shadow_pass:{value:null},resolution:{value:new ze},radius:{value:4}},vertexShader:oT,fragmentShader:aT}),m=f.clone();m.defines.HORIZONTAL_PASS=1;let _=new pi;_.setAttribute("position",new nn(new Float32Array([-1,-1,.5,3,-1,.5,-1,3,.5]),3));let y=new St(_,f),d=this;this.enabled=!1,this.autoUpdate=!0,this.needsUpdate=!1,this.type=mv;let h=this.type;this.render=function(C,T,L){if(d.enabled===!1||d.autoUpdate===!1&&d.needsUpdate===!1||C.length===0)return;let E=n.getRenderTarget(),M=n.getActiveCubeFace(),P=n.getActiveMipmapLevel(),W=n.state;W.setBlending(Yi),W.buffers.color.setClear(1,1,1,1),W.buffers.depth.setTest(!0),W.setScissorTest(!1);let k=h!==ai&&this.type===ai,G=h===ai&&this.type!==ai;for(let $=0,V=C.length;$<V;$++){let K=C[$],z=K.shadow;if(z===void 0){console.warn("THREE.WebGLShadowMap:",K,"has no shadow.");continue}if(z.autoUpdate===!1&&z.needsUpdate===!1)continue;r.copy(z.mapSize);let he=z.getFrameExtents();if(r.multiply(he),s.copy(z.mapSize),(r.x>u||r.y>u)&&(r.x>u&&(s.x=Math.floor(u/he.x),r.x=s.x*he.x,z.mapSize.x=s.x),r.y>u&&(s.y=Math.floor(u/he.y),r.y=s.y*he.y,z.mapSize.y=s.y)),z.map===null||k===!0||G===!0){let _e=this.type!==ai?{minFilter:Sn,magFilter:Sn}:{};z.map!==null&&z.map.dispose(),z.map=new fi(r.x,r.y,_e),z.map.texture.name=K.name+".shadowMap",z.camera.updateProjectionMatrix()}n.setRenderTarget(z.map),n.clear();let ge=z.getViewportCount();for(let _e=0;_e<ge;_e++){let Ve=z.getViewport(_e);o.set(s.x*Ve.x,s.y*Ve.y,s.x*Ve.z,s.y*Ve.w),W.viewport(o),z.updateMatrices(K,_e),i=z.getFrustum(),x(T,L,z.camera,K,this.type)}z.isPointLightShadow!==!0&&this.type===ai&&g(z,L),z.needsUpdate=!1}h=this.type,d.needsUpdate=!1,n.setRenderTarget(E,M,P)};function g(C,T){let L=e.update(y);f.defines.VSM_SAMPLES!==C.blurSamples&&(f.defines.VSM_SAMPLES=C.blurSamples,m.defines.VSM_SAMPLES=C.blurSamples,f.needsUpdate=!0,m.needsUpdate=!0),C.mapPass===null&&(C.mapPass=new fi(r.x,r.y)),f.uniforms.shadow_pass.value=C.map.texture,f.uniforms.resolution.value=C.mapSize,f.uniforms.radius.value=C.radius,n.setRenderTarget(C.mapPass),n.clear(),n.renderBufferDirect(T,null,L,f,y,null),m.uniforms.shadow_pass.value=C.mapPass.texture,m.uniforms.resolution.value=C.mapSize,m.uniforms.radius.value=C.radius,n.setRenderTarget(C.map),n.clear(),n.renderBufferDirect(T,null,L,m,y,null)}function v(C,T,L,E){let M=null,P=L.isPointLight===!0?C.customDistanceMaterial:C.customDepthMaterial;if(P!==void 0)M=P;else if(M=L.isPointLight===!0?l:a,n.localClippingEnabled&&T.clipShadows===!0&&Array.isArray(T.clippingPlanes)&&T.clippingPlanes.length!==0||T.displacementMap&&T.displacementScale!==0||T.alphaMap&&T.alphaTest>0||T.map&&T.alphaTest>0){let W=M.uuid,k=T.uuid,G=c[W];G===void 0&&(G={},c[W]=G);let $=G[k];$===void 0&&($=M.clone(),G[k]=$,T.addEventListener("dispose",R)),M=$}if(M.visible=T.visible,M.wireframe=T.wireframe,E===ai?M.side=T.shadowSide!==null?T.shadowSide:T.side:M.side=T.shadowSide!==null?T.shadowSide:p[T.side],M.alphaMap=T.alphaMap,M.alphaTest=T.alphaTest,M.map=T.map,M.clipShadows=T.clipShadows,M.clippingPlanes=T.clippingPlanes,M.clipIntersection=T.clipIntersection,M.displacementMap=T.displacementMap,M.displacementScale=T.displacementScale,M.displacementBias=T.displacementBias,M.wireframeLinewidth=T.wireframeLinewidth,M.linewidth=T.linewidth,L.isPointLight===!0&&M.isMeshDistanceMaterial===!0){let W=n.properties.get(M);W.light=L}return M}function x(C,T,L,E,M){if(C.visible===!1)return;if(C.layers.test(T.layers)&&(C.isMesh||C.isLine||C.isPoints)&&(C.castShadow||C.receiveShadow&&M===ai)&&(!C.frustumCulled||i.intersectsObject(C))){C.modelViewMatrix.multiplyMatrices(L.matrixWorldInverse,C.matrixWorld);let k=e.update(C),G=C.material;if(Array.isArray(G)){let $=k.groups;for(let V=0,K=$.length;V<K;V++){let z=$[V],he=G[z.materialIndex];if(he&&he.visible){let ge=v(C,he,E,M);C.onBeforeShadow(n,C,T,L,k,ge,z),n.renderBufferDirect(L,null,k,ge,C,z),C.onAfterShadow(n,C,T,L,k,ge,z)}}}else if(G.visible){let $=v(C,G,E,M);C.onBeforeShadow(n,C,T,L,k,$,null),n.renderBufferDirect(L,null,k,$,C,null),C.onAfterShadow(n,C,T,L,k,$,null)}}let W=C.children;for(let k=0,G=W.length;k<G;k++)x(W[k],T,L,E,M)}function R(C){C.target.removeEventListener("dispose",R);for(let L in c){let E=c[L],M=C.target.uuid;M in E&&(E[M].dispose(),delete E[M])}}}function cT(n){function e(){let I=!1,te=new wt,X=null,Y=new wt(0,0,0,0);return{setMask:function(ie){X!==ie&&!I&&(n.colorMask(ie,ie,ie,ie),X=ie)},setLocked:function(ie){I=ie},setClear:function(ie,Se,Ge,gt,bt){bt===!0&&(ie*=gt,Se*=gt,Ge*=gt),te.set(ie,Se,Ge,gt),Y.equals(te)===!1&&(n.clearColor(ie,Se,Ge,gt),Y.copy(te))},reset:function(){I=!1,X=null,Y.set(-1,0,0,0)}}}function t(){let I=!1,te=null,X=null,Y=null;return{setTest:function(ie){ie?fe(n.DEPTH_TEST):ce(n.DEPTH_TEST)},setMask:function(ie){te!==ie&&!I&&(n.depthMask(ie),te=ie)},setFunc:function(ie){if(X!==ie){switch(ie){case Kw:n.depthFunc(n.NEVER);break;case jw:n.depthFunc(n.ALWAYS);break;case Qw:n.depthFunc(n.LESS);break;case oc:n.depthFunc(n.LEQUAL);break;case eS:n.depthFunc(n.EQUAL);break;case tS:n.depthFunc(n.GEQUAL);break;case nS:n.depthFunc(n.GREATER);break;case iS:n.depthFunc(n.NOTEQUAL);break;default:n.depthFunc(n.LEQUAL)}X=ie}},setLocked:function(ie){I=ie},setClear:function(ie){Y!==ie&&(n.clearDepth(ie),Y=ie)},reset:function(){I=!1,te=null,X=null,Y=null}}}function i(){let I=!1,te=null,X=null,Y=null,ie=null,Se=null,Ge=null,gt=null,bt=null;return{setTest:function(Xe){I||(Xe?fe(n.STENCIL_TEST):ce(n.STENCIL_TEST))},setMask:function(Xe){te!==Xe&&!I&&(n.stencilMask(Xe),te=Xe)},setFunc:function(Xe,qn,On){(X!==Xe||Y!==qn||ie!==On)&&(n.stencilFunc(Xe,qn,On),X=Xe,Y=qn,ie=On)},setOp:function(Xe,qn,On){(Se!==Xe||Ge!==qn||gt!==On)&&(n.stencilOp(Xe,qn,On),Se=Xe,Ge=qn,gt=On)},setLocked:function(Xe){I=Xe},setClear:function(Xe){bt!==Xe&&(n.clearStencil(Xe),bt=Xe)},reset:function(){I=!1,te=null,X=null,Y=null,ie=null,Se=null,Ge=null,gt=null,bt=null}}}let r=new e,s=new t,o=new i,a=new WeakMap,l=new WeakMap,c={},u={},p=new WeakMap,f=[],m=null,_=!1,y=null,d=null,h=null,g=null,v=null,x=null,R=null,C=new Oe(0,0,0),T=0,L=!1,E=null,M=null,P=null,W=null,k=null,G=n.getParameter(n.MAX_COMBINED_TEXTURE_IMAGE_UNITS),$=!1,V=0,K=n.getParameter(n.VERSION);K.indexOf("WebGL")!==-1?(V=parseFloat(/^WebGL (\d)/.exec(K)[1]),$=V>=1):K.indexOf("OpenGL ES")!==-1&&(V=parseFloat(/^OpenGL ES (\d)/.exec(K)[1]),$=V>=2);let z=null,he={},ge=n.getParameter(n.SCISSOR_BOX),_e=n.getParameter(n.VIEWPORT),Ve=new wt().fromArray(ge),je=new wt().fromArray(_e);function H(I,te,X,Y){let ie=new Uint8Array(4),Se=n.createTexture();n.bindTexture(I,Se),n.texParameteri(I,n.TEXTURE_MIN_FILTER,n.NEAREST),n.texParameteri(I,n.TEXTURE_MAG_FILTER,n.NEAREST);for(let Ge=0;Ge<X;Ge++)I===n.TEXTURE_3D||I===n.TEXTURE_2D_ARRAY?n.texImage3D(te,0,n.RGBA,1,1,Y,0,n.RGBA,n.UNSIGNED_BYTE,ie):n.texImage2D(te+Ge,0,n.RGBA,1,1,0,n.RGBA,n.UNSIGNED_BYTE,ie);return Se}let Q={};Q[n.TEXTURE_2D]=H(n.TEXTURE_2D,n.TEXTURE_2D,1),Q[n.TEXTURE_CUBE_MAP]=H(n.TEXTURE_CUBE_MAP,n.TEXTURE_CUBE_MAP_POSITIVE_X,6),Q[n.TEXTURE_2D_ARRAY]=H(n.TEXTURE_2D_ARRAY,n.TEXTURE_2D_ARRAY,1,1),Q[n.TEXTURE_3D]=H(n.TEXTURE_3D,n.TEXTURE_3D,1,1),r.setClear(0,0,0,1),s.setClear(1),o.setClear(0),fe(n.DEPTH_TEST),s.setFunc(oc),Qe(!1),ye(m_),fe(n.CULL_FACE),pt(Yi);function fe(I){c[I]!==!0&&(n.enable(I),c[I]=!0)}function ce(I){c[I]!==!1&&(n.disable(I),c[I]=!1)}function Re(I,te){return u[I]!==te?(n.bindFramebuffer(I,te),u[I]=te,I===n.DRAW_FRAMEBUFFER&&(u[n.FRAMEBUFFER]=te),I===n.FRAMEBUFFER&&(u[n.DRAW_FRAMEBUFFER]=te),!0):!1}function Ne(I,te){let X=f,Y=!1;if(I){X=p.get(te),X===void 0&&(X=[],p.set(te,X));let ie=I.textures;if(X.length!==ie.length||X[0]!==n.COLOR_ATTACHMENT0){for(let Se=0,Ge=ie.length;Se<Ge;Se++)X[Se]=n.COLOR_ATTACHMENT0+Se;X.length=ie.length,Y=!0}}else X[0]!==n.BACK&&(X[0]=n.BACK,Y=!0);Y&&n.drawBuffers(X)}function Be(I){return m!==I?(n.useProgram(I),m=I,!0):!1}let lt={[Sr]:n.FUNC_ADD,[Nw]:n.FUNC_SUBTRACT,[Dw]:n.FUNC_REVERSE_SUBTRACT};lt[Ow]=n.MIN,lt[Fw]=n.MAX;let A={[Bw]:n.ZERO,[kw]:n.ONE,[zw]:n.SRC_COLOR,[tf]:n.SRC_ALPHA,[Yw]:n.SRC_ALPHA_SATURATE,[Ww]:n.DST_COLOR,[Hw]:n.DST_ALPHA,[Vw]:n.ONE_MINUS_SRC_COLOR,[nf]:n.ONE_MINUS_SRC_ALPHA,[Xw]:n.ONE_MINUS_DST_COLOR,[Gw]:n.ONE_MINUS_DST_ALPHA,[qw]:n.CONSTANT_COLOR,[$w]:n.ONE_MINUS_CONSTANT_COLOR,[Zw]:n.CONSTANT_ALPHA,[Jw]:n.ONE_MINUS_CONSTANT_ALPHA};function pt(I,te,X,Y,ie,Se,Ge,gt,bt,Xe){if(I===Yi){_===!0&&(ce(n.BLEND),_=!1);return}if(_===!1&&(fe(n.BLEND),_=!0),I!==Uw){if(I!==y||Xe!==L){if((d!==Sr||v!==Sr)&&(n.blendEquation(n.FUNC_ADD),d=Sr,v=Sr),Xe)switch(I){case Us:n.blendFuncSeparate(n.ONE,n.ONE_MINUS_SRC_ALPHA,n.ONE,n.ONE_MINUS_SRC_ALPHA);break;case g_:n.blendFunc(n.ONE,n.ONE);break;case __:n.blendFuncSeparate(n.ZERO,n.ONE_MINUS_SRC_COLOR,n.ZERO,n.ONE);break;case v_:n.blendFuncSeparate(n.ZERO,n.SRC_COLOR,n.ZERO,n.SRC_ALPHA);break;default:console.error("THREE.WebGLState: Invalid blending: ",I);break}else switch(I){case Us:n.blendFuncSeparate(n.SRC_ALPHA,n.ONE_MINUS_SRC_ALPHA,n.ONE,n.ONE_MINUS_SRC_ALPHA);break;case g_:n.blendFunc(n.SRC_ALPHA,n.ONE);break;case __:n.blendFuncSeparate(n.ZERO,n.ONE_MINUS_SRC_COLOR,n.ZERO,n.ONE);break;case v_:n.blendFunc(n.ZERO,n.SRC_COLOR);break;default:console.error("THREE.WebGLState: Invalid blending: ",I);break}h=null,g=null,x=null,R=null,C.set(0,0,0),T=0,y=I,L=Xe}return}ie=ie||te,Se=Se||X,Ge=Ge||Y,(te!==d||ie!==v)&&(n.blendEquationSeparate(lt[te],lt[ie]),d=te,v=ie),(X!==h||Y!==g||Se!==x||Ge!==R)&&(n.blendFuncSeparate(A[X],A[Y],A[Se],A[Ge]),h=X,g=Y,x=Se,R=Ge),(gt.equals(C)===!1||bt!==T)&&(n.blendColor(gt.r,gt.g,gt.b,bt),C.copy(gt),T=bt),y=I,L=!1}function Je(I,te){I.side===li?ce(n.CULL_FACE):fe(n.CULL_FACE);let X=I.side===tn;te&&(X=!X),Qe(X),I.blending===Us&&I.transparent===!1?pt(Yi):pt(I.blending,I.blendEquation,I.blendSrc,I.blendDst,I.blendEquationAlpha,I.blendSrcAlpha,I.blendDstAlpha,I.blendColor,I.blendAlpha,I.premultipliedAlpha),s.setFunc(I.depthFunc),s.setTest(I.depthTest),s.setMask(I.depthWrite),r.setMask(I.colorWrite);let Y=I.stencilWrite;o.setTest(Y),Y&&(o.setMask(I.stencilWriteMask),o.setFunc(I.stencilFunc,I.stencilRef,I.stencilFuncMask),o.setOp(I.stencilFail,I.stencilZFail,I.stencilZPass)),be(I.polygonOffset,I.polygonOffsetFactor,I.polygonOffsetUnits),I.alphaToCoverage===!0?fe(n.SAMPLE_ALPHA_TO_COVERAGE):ce(n.SAMPLE_ALPHA_TO_COVERAGE)}function Qe(I){E!==I&&(I?n.frontFace(n.CW):n.frontFace(n.CCW),E=I)}function ye(I){I!==Iw?(fe(n.CULL_FACE),I!==M&&(I===m_?n.cullFace(n.BACK):I===Lw?n.cullFace(n.FRONT):n.cullFace(n.FRONT_AND_BACK))):ce(n.CULL_FACE),M=I}function mt(I){I!==P&&($&&n.lineWidth(I),P=I)}function be(I,te,X){I?(fe(n.POLYGON_OFFSET_FILL),(W!==te||k!==X)&&(n.polygonOffset(te,X),W=te,k=X)):ce(n.POLYGON_OFFSET_FILL)}function Pe(I){I?fe(n.SCISSOR_TEST):ce(n.SCISSOR_TEST)}function b(I){I===void 0&&(I=n.TEXTURE0+G-1),z!==I&&(n.activeTexture(I),z=I)}function w(I,te,X){X===void 0&&(z===null?X=n.TEXTURE0+G-1:X=z);let Y=he[X];Y===void 0&&(Y={type:void 0,texture:void 0},he[X]=Y),(Y.type!==I||Y.texture!==te)&&(z!==X&&(n.activeTexture(X),z=X),n.bindTexture(I,te||Q[I]),Y.type=I,Y.texture=te)}function B(){let I=he[z];I!==void 0&&I.type!==void 0&&(n.bindTexture(I.type,null),I.type=void 0,I.texture=void 0)}function Z(){try{n.compressedTexImage2D.apply(n,arguments)}catch(I){console.error("THREE.WebGLState:",I)}}function J(){try{n.compressedTexImage3D.apply(n,arguments)}catch(I){console.error("THREE.WebGLState:",I)}}function q(){try{n.texSubImage2D.apply(n,arguments)}catch(I){console.error("THREE.WebGLState:",I)}}function xe(){try{n.texSubImage3D.apply(n,arguments)}catch(I){console.error("THREE.WebGLState:",I)}}function se(){try{n.compressedTexSubImage2D.apply(n,arguments)}catch(I){console.error("THREE.WebGLState:",I)}}function ue(){try{n.compressedTexSubImage3D.apply(n,arguments)}catch(I){console.error("THREE.WebGLState:",I)}}function Ie(){try{n.texStorage2D.apply(n,arguments)}catch(I){console.error("THREE.WebGLState:",I)}}function ee(){try{n.texStorage3D.apply(n,arguments)}catch(I){console.error("THREE.WebGLState:",I)}}function ae(){try{n.texImage2D.apply(n,arguments)}catch(I){console.error("THREE.WebGLState:",I)}}function He(){try{n.texImage3D.apply(n,arguments)}catch(I){console.error("THREE.WebGLState:",I)}}function Te(I){Ve.equals(I)===!1&&(n.scissor(I.x,I.y,I.z,I.w),Ve.copy(I))}function de(I){je.equals(I)===!1&&(n.viewport(I.x,I.y,I.z,I.w),je.copy(I))}function Ae(I,te){let X=l.get(te);X===void 0&&(X=new WeakMap,l.set(te,X));let Y=X.get(I);Y===void 0&&(Y=n.getUniformBlockIndex(te,I.name),X.set(I,Y))}function De(I,te){let Y=l.get(te).get(I);a.get(te)!==Y&&(n.uniformBlockBinding(te,Y,I.__bindingPointIndex),a.set(te,Y))}function at(){n.disable(n.BLEND),n.disable(n.CULL_FACE),n.disable(n.DEPTH_TEST),n.disable(n.POLYGON_OFFSET_FILL),n.disable(n.SCISSOR_TEST),n.disable(n.STENCIL_TEST),n.disable(n.SAMPLE_ALPHA_TO_COVERAGE),n.blendEquation(n.FUNC_ADD),n.blendFunc(n.ONE,n.ZERO),n.blendFuncSeparate(n.ONE,n.ZERO,n.ONE,n.ZERO),n.blendColor(0,0,0,0),n.colorMask(!0,!0,!0,!0),n.clearColor(0,0,0,0),n.depthMask(!0),n.depthFunc(n.LESS),n.clearDepth(1),n.stencilMask(4294967295),n.stencilFunc(n.ALWAYS,0,4294967295),n.stencilOp(n.KEEP,n.KEEP,n.KEEP),n.clearStencil(0),n.cullFace(n.BACK),n.frontFace(n.CCW),n.polygonOffset(0,0),n.activeTexture(n.TEXTURE0),n.bindFramebuffer(n.FRAMEBUFFER,null),n.bindFramebuffer(n.DRAW_FRAMEBUFFER,null),n.bindFramebuffer(n.READ_FRAMEBUFFER,null),n.useProgram(null),n.lineWidth(1),n.scissor(0,0,n.canvas.width,n.canvas.height),n.viewport(0,0,n.canvas.width,n.canvas.height),c={},z=null,he={},u={},p=new WeakMap,f=[],m=null,_=!1,y=null,d=null,h=null,g=null,v=null,x=null,R=null,C=new Oe(0,0,0),T=0,L=!1,E=null,M=null,P=null,W=null,k=null,Ve.set(0,0,n.canvas.width,n.canvas.height),je.set(0,0,n.canvas.width,n.canvas.height),r.reset(),s.reset(),o.reset()}return{buffers:{color:r,depth:s,stencil:o},enable:fe,disable:ce,bindFramebuffer:Re,drawBuffers:Ne,useProgram:Be,setBlending:pt,setMaterial:Je,setFlipSided:Qe,setCullFace:ye,setLineWidth:mt,setPolygonOffset:be,setScissorTest:Pe,activeTexture:b,bindTexture:w,unbindTexture:B,compressedTexImage2D:Z,compressedTexImage3D:J,texImage2D:ae,texImage3D:He,updateUBOMapping:Ae,uniformBlockBinding:De,texStorage2D:Ie,texStorage3D:ee,texSubImage2D:q,texSubImage3D:xe,compressedTexSubImage2D:se,compressedTexSubImage3D:ue,scissor:Te,viewport:de,reset:at}}function dv(n,e,t,i){let r=uT(i);switch(t){case wv:return n*e;case Mv:return n*e;case Ev:return n*e*2;case Cv:return n*e/r.components*r.byteLength;case _p:return n*e/r.components*r.byteLength;case Tv:return n*e*2/r.components*r.byteLength;case vp:return n*e*2/r.components*r.byteLength;case Sv:return n*e*3/r.components*r.byteLength;case Un:return n*e*4/r.components*r.byteLength;case yp:return n*e*4/r.components*r.byteLength;case tc:case nc:return Math.floor((n+3)/4)*Math.floor((e+3)/4)*8;case ic:case rc:return Math.floor((n+3)/4)*Math.floor((e+3)/4)*16;case cf:case hf:return Math.max(n,16)*Math.max(e,8)/4;case lf:case uf:return Math.max(n,8)*Math.max(e,8)/2;case df:case ff:return Math.floor((n+3)/4)*Math.floor((e+3)/4)*8;case pf:return Math.floor((n+3)/4)*Math.floor((e+3)/4)*16;case mf:return Math.floor((n+3)/4)*Math.floor((e+3)/4)*16;case gf:return Math.floor((n+4)/5)*Math.floor((e+3)/4)*16;case _f:return Math.floor((n+4)/5)*Math.floor((e+4)/5)*16;case vf:return Math.floor((n+5)/6)*Math.floor((e+4)/5)*16;case yf:return Math.floor((n+5)/6)*Math.floor((e+5)/6)*16;case xf:return Math.floor((n+7)/8)*Math.floor((e+4)/5)*16;case wf:return Math.floor((n+7)/8)*Math.floor((e+5)/6)*16;case Sf:return Math.floor((n+7)/8)*Math.floor((e+7)/8)*16;case Mf:return Math.floor((n+9)/10)*Math.floor((e+4)/5)*16;case Ef:return Math.floor((n+9)/10)*Math.floor((e+5)/6)*16;case Cf:return Math.floor((n+9)/10)*Math.floor((e+7)/8)*16;case Tf:return Math.floor((n+9)/10)*Math.floor((e+9)/10)*16;case bf:return Math.floor((n+11)/12)*Math.floor((e+9)/10)*16;case Af:return Math.floor((n+11)/12)*Math.floor((e+11)/12)*16;case sc:case Rf:case Pf:return Math.ceil(n/4)*Math.ceil(e/4)*16;case bv:case If:return Math.ceil(n/4)*Math.ceil(e/4)*8;case Lf:case Uf:return Math.ceil(n/4)*Math.ceil(e/4)*16}throw new Error(`Unable to determine texture byte length for ${t} format.`)}function uT(n){switch(n){case di:case vv:return{byteLength:1,components:1};case ta:case yv:case aa:return{byteLength:2,components:1};case mp:case gp:return{byteLength:2,components:4};case Tr:case pp:case ci:return{byteLength:4,components:1};case xv:return{byteLength:4,components:3}}throw new Error(`Unknown texture type ${n}.`)}function hT(n,e,t,i,r,s,o){let a=e.has("WEBGL_multisampled_render_to_texture")?e.get("WEBGL_multisampled_render_to_texture"):null,l=typeof navigator>"u"?!1:/OculusBrowser/g.test(navigator.userAgent),c=new ze,u=new WeakMap,p,f=new WeakMap,m=!1;try{m=typeof OffscreenCanvas<"u"&&new OffscreenCanvas(1,1).getContext("2d")!==null}catch{}function _(b,w){return m?new OffscreenCanvas(b,w):na("canvas")}function y(b,w,B){let Z=1,J=Pe(b);if((J.width>B||J.height>B)&&(Z=B/Math.max(J.width,J.height)),Z<1)if(typeof HTMLImageElement<"u"&&b instanceof HTMLImageElement||typeof HTMLCanvasElement<"u"&&b instanceof HTMLCanvasElement||typeof ImageBitmap<"u"&&b instanceof ImageBitmap||typeof VideoFrame<"u"&&b instanceof VideoFrame){let q=Math.floor(Z*J.width),xe=Math.floor(Z*J.height);p===void 0&&(p=_(q,xe));let se=w?_(q,xe):p;return se.width=q,se.height=xe,se.getContext("2d").drawImage(b,0,0,q,xe),console.warn("THREE.WebGLRenderer: Texture has been resized from ("+J.width+"x"+J.height+") to ("+q+"x"+xe+")."),se}else return"data"in b&&console.warn("THREE.WebGLRenderer: Image in DataTexture is too big ("+J.width+"x"+J.height+")."),b;return b}function d(b){return b.generateMipmaps&&b.minFilter!==Sn&&b.minFilter!==qt}function h(b){n.generateMipmap(b)}function g(b,w,B,Z,J=!1){if(b!==null){if(n[b]!==void 0)return n[b];console.warn("THREE.WebGLRenderer: Attempt to use non-existing WebGL internal format '"+b+"'")}let q=w;if(w===n.RED&&(B===n.FLOAT&&(q=n.R32F),B===n.HALF_FLOAT&&(q=n.R16F),B===n.UNSIGNED_BYTE&&(q=n.R8)),w===n.RED_INTEGER&&(B===n.UNSIGNED_BYTE&&(q=n.R8UI),B===n.UNSIGNED_SHORT&&(q=n.R16UI),B===n.UNSIGNED_INT&&(q=n.R32UI),B===n.BYTE&&(q=n.R8I),B===n.SHORT&&(q=n.R16I),B===n.INT&&(q=n.R32I)),w===n.RG&&(B===n.FLOAT&&(q=n.RG32F),B===n.HALF_FLOAT&&(q=n.RG16F),B===n.UNSIGNED_BYTE&&(q=n.RG8)),w===n.RG_INTEGER&&(B===n.UNSIGNED_BYTE&&(q=n.RG8UI),B===n.UNSIGNED_SHORT&&(q=n.RG16UI),B===n.UNSIGNED_INT&&(q=n.RG32UI),B===n.BYTE&&(q=n.RG8I),B===n.SHORT&&(q=n.RG16I),B===n.INT&&(q=n.RG32I)),w===n.RGB&&B===n.UNSIGNED_INT_5_9_9_9_REV&&(q=n.RGB9_E5),w===n.RGBA){let xe=J?lc:Ke.getTransfer(Z);B===n.FLOAT&&(q=n.RGBA32F),B===n.HALF_FLOAT&&(q=n.RGBA16F),B===n.UNSIGNED_BYTE&&(q=xe===rt?n.SRGB8_ALPHA8:n.RGBA8),B===n.UNSIGNED_SHORT_4_4_4_4&&(q=n.RGBA4),B===n.UNSIGNED_SHORT_5_5_5_1&&(q=n.RGB5_A1)}return(q===n.R16F||q===n.R32F||q===n.RG16F||q===n.RG32F||q===n.RGBA16F||q===n.RGBA32F)&&e.get("EXT_color_buffer_float"),q}function v(b,w){let B;return b?w===null||w===Tr||w===ks?B=n.DEPTH24_STENCIL8:w===ci?B=n.DEPTH32F_STENCIL8:w===ta&&(B=n.DEPTH24_STENCIL8,console.warn("DepthTexture: 16 bit depth attachment is not supported with stencil. Using 24-bit attachment.")):w===null||w===Tr||w===ks?B=n.DEPTH_COMPONENT24:w===ci?B=n.DEPTH_COMPONENT32F:w===ta&&(B=n.DEPTH_COMPONENT16),B}function x(b,w){return d(b)===!0||b.isFramebufferTexture&&b.minFilter!==Sn&&b.minFilter!==qt?Math.log2(Math.max(w.width,w.height))+1:b.mipmaps!==void 0&&b.mipmaps.length>0?b.mipmaps.length:b.isCompressedTexture&&Array.isArray(b.image)?w.mipmaps.length:1}function R(b){let w=b.target;w.removeEventListener("dispose",R),T(w),w.isVideoTexture&&u.delete(w)}function C(b){let w=b.target;w.removeEventListener("dispose",C),E(w)}function T(b){let w=i.get(b);if(w.__webglInit===void 0)return;let B=b.source,Z=f.get(B);if(Z){let J=Z[w.__cacheKey];J.usedTimes--,J.usedTimes===0&&L(b),Object.keys(Z).length===0&&f.delete(B)}i.remove(b)}function L(b){let w=i.get(b);n.deleteTexture(w.__webglTexture);let B=b.source,Z=f.get(B);delete Z[w.__cacheKey],o.memory.textures--}function E(b){let w=i.get(b);if(b.depthTexture&&b.depthTexture.dispose(),b.isWebGLCubeRenderTarget)for(let Z=0;Z<6;Z++){if(Array.isArray(w.__webglFramebuffer[Z]))for(let J=0;J<w.__webglFramebuffer[Z].length;J++)n.deleteFramebuffer(w.__webglFramebuffer[Z][J]);else n.deleteFramebuffer(w.__webglFramebuffer[Z]);w.__webglDepthbuffer&&n.deleteRenderbuffer(w.__webglDepthbuffer[Z])}else{if(Array.isArray(w.__webglFramebuffer))for(let Z=0;Z<w.__webglFramebuffer.length;Z++)n.deleteFramebuffer(w.__webglFramebuffer[Z]);else n.deleteFramebuffer(w.__webglFramebuffer);if(w.__webglDepthbuffer&&n.deleteRenderbuffer(w.__webglDepthbuffer),w.__webglMultisampledFramebuffer&&n.deleteFramebuffer(w.__webglMultisampledFramebuffer),w.__webglColorRenderbuffer)for(let Z=0;Z<w.__webglColorRenderbuffer.length;Z++)w.__webglColorRenderbuffer[Z]&&n.deleteRenderbuffer(w.__webglColorRenderbuffer[Z]);w.__webglDepthRenderbuffer&&n.deleteRenderbuffer(w.__webglDepthRenderbuffer)}let B=b.textures;for(let Z=0,J=B.length;Z<J;Z++){let q=i.get(B[Z]);q.__webglTexture&&(n.deleteTexture(q.__webglTexture),o.memory.textures--),i.remove(B[Z])}i.remove(b)}let M=0;function P(){M=0}function W(){let b=M;return b>=r.maxTextures&&console.warn("THREE.WebGLTextures: Trying to use "+b+" texture units while this GPU supports only "+r.maxTextures),M+=1,b}function k(b){let w=[];return w.push(b.wrapS),w.push(b.wrapT),w.push(b.wrapR||0),w.push(b.magFilter),w.push(b.minFilter),w.push(b.anisotropy),w.push(b.internalFormat),w.push(b.format),w.push(b.type),w.push(b.generateMipmaps),w.push(b.premultiplyAlpha),w.push(b.flipY),w.push(b.unpackAlignment),w.push(b.colorSpace),w.join()}function G(b,w){let B=i.get(b);if(b.isVideoTexture&&mt(b),b.isRenderTargetTexture===!1&&b.version>0&&B.__version!==b.version){let Z=b.image;if(Z===null)console.warn("THREE.WebGLRenderer: Texture marked for update but no image data found.");else if(Z.complete===!1)console.warn("THREE.WebGLRenderer: Texture marked for update but image is incomplete");else{je(B,b,w);return}}t.bindTexture(n.TEXTURE_2D,B.__webglTexture,n.TEXTURE0+w)}function $(b,w){let B=i.get(b);if(b.version>0&&B.__version!==b.version){je(B,b,w);return}t.bindTexture(n.TEXTURE_2D_ARRAY,B.__webglTexture,n.TEXTURE0+w)}function V(b,w){let B=i.get(b);if(b.version>0&&B.__version!==b.version){je(B,b,w);return}t.bindTexture(n.TEXTURE_3D,B.__webglTexture,n.TEXTURE0+w)}function K(b,w){let B=i.get(b);if(b.version>0&&B.__version!==b.version){H(B,b,w);return}t.bindTexture(n.TEXTURE_CUBE_MAP,B.__webglTexture,n.TEXTURE0+w)}let z={[of]:n.REPEAT,[Er]:n.CLAMP_TO_EDGE,[af]:n.MIRRORED_REPEAT},he={[Sn]:n.NEAREST,[fS]:n.NEAREST_MIPMAP_NEAREST,[Ll]:n.NEAREST_MIPMAP_LINEAR,[qt]:n.LINEAR,[Td]:n.LINEAR_MIPMAP_NEAREST,[Cr]:n.LINEAR_MIPMAP_LINEAR},ge={[_S]:n.NEVER,[MS]:n.ALWAYS,[vS]:n.LESS,[Rv]:n.LEQUAL,[yS]:n.EQUAL,[SS]:n.GEQUAL,[xS]:n.GREATER,[wS]:n.NOTEQUAL};function _e(b,w){if(w.type===ci&&e.has("OES_texture_float_linear")===!1&&(w.magFilter===qt||w.magFilter===Td||w.magFilter===Ll||w.magFilter===Cr||w.minFilter===qt||w.minFilter===Td||w.minFilter===Ll||w.minFilter===Cr)&&console.warn("THREE.WebGLRenderer: Unable to use linear filtering with floating point textures. OES_texture_float_linear not supported on this device."),n.texParameteri(b,n.TEXTURE_WRAP_S,z[w.wrapS]),n.texParameteri(b,n.TEXTURE_WRAP_T,z[w.wrapT]),(b===n.TEXTURE_3D||b===n.TEXTURE_2D_ARRAY)&&n.texParameteri(b,n.TEXTURE_WRAP_R,z[w.wrapR]),n.texParameteri(b,n.TEXTURE_MAG_FILTER,he[w.magFilter]),n.texParameteri(b,n.TEXTURE_MIN_FILTER,he[w.minFilter]),w.compareFunction&&(n.texParameteri(b,n.TEXTURE_COMPARE_MODE,n.COMPARE_REF_TO_TEXTURE),n.texParameteri(b,n.TEXTURE_COMPARE_FUNC,ge[w.compareFunction])),e.has("EXT_texture_filter_anisotropic")===!0){if(w.magFilter===Sn||w.minFilter!==Ll&&w.minFilter!==Cr||w.type===ci&&e.has("OES_texture_float_linear")===!1)return;if(w.anisotropy>1||i.get(w).__currentAnisotropy){let B=e.get("EXT_texture_filter_anisotropic");n.texParameterf(b,B.TEXTURE_MAX_ANISOTROPY_EXT,Math.min(w.anisotropy,r.getMaxAnisotropy())),i.get(w).__currentAnisotropy=w.anisotropy}}}function Ve(b,w){let B=!1;b.__webglInit===void 0&&(b.__webglInit=!0,w.addEventListener("dispose",R));let Z=w.source,J=f.get(Z);J===void 0&&(J={},f.set(Z,J));let q=k(w);if(q!==b.__cacheKey){J[q]===void 0&&(J[q]={texture:n.createTexture(),usedTimes:0},o.memory.textures++,B=!0),J[q].usedTimes++;let xe=J[b.__cacheKey];xe!==void 0&&(J[b.__cacheKey].usedTimes--,xe.usedTimes===0&&L(w)),b.__cacheKey=q,b.__webglTexture=J[q].texture}return B}function je(b,w,B){let Z=n.TEXTURE_2D;(w.isDataArrayTexture||w.isCompressedArrayTexture)&&(Z=n.TEXTURE_2D_ARRAY),w.isData3DTexture&&(Z=n.TEXTURE_3D);let J=Ve(b,w),q=w.source;t.bindTexture(Z,b.__webglTexture,n.TEXTURE0+B);let xe=i.get(q);if(q.version!==xe.__version||J===!0){t.activeTexture(n.TEXTURE0+B);let se=Ke.getPrimaries(Ke.workingColorSpace),ue=w.colorSpace===Wi?null:Ke.getPrimaries(w.colorSpace),Ie=w.colorSpace===Wi||se===ue?n.NONE:n.BROWSER_DEFAULT_WEBGL;n.pixelStorei(n.UNPACK_FLIP_Y_WEBGL,w.flipY),n.pixelStorei(n.UNPACK_PREMULTIPLY_ALPHA_WEBGL,w.premultiplyAlpha),n.pixelStorei(n.UNPACK_ALIGNMENT,w.unpackAlignment),n.pixelStorei(n.UNPACK_COLORSPACE_CONVERSION_WEBGL,Ie);let ee=y(w.image,!1,r.maxTextureSize);ee=be(w,ee);let ae=s.convert(w.format,w.colorSpace),He=s.convert(w.type),Te=g(w.internalFormat,ae,He,w.colorSpace,w.isVideoTexture);_e(Z,w);let de,Ae=w.mipmaps,De=w.isVideoTexture!==!0,at=xe.__version===void 0||J===!0,I=q.dataReady,te=x(w,ee);if(w.isDepthTexture)Te=v(w.format===zs,w.type),at&&(De?t.texStorage2D(n.TEXTURE_2D,1,Te,ee.width,ee.height):t.texImage2D(n.TEXTURE_2D,0,Te,ee.width,ee.height,0,ae,He,null));else if(w.isDataTexture)if(Ae.length>0){De&&at&&t.texStorage2D(n.TEXTURE_2D,te,Te,Ae[0].width,Ae[0].height);for(let X=0,Y=Ae.length;X<Y;X++)de=Ae[X],De?I&&t.texSubImage2D(n.TEXTURE_2D,X,0,0,de.width,de.height,ae,He,de.data):t.texImage2D(n.TEXTURE_2D,X,Te,de.width,de.height,0,ae,He,de.data);w.generateMipmaps=!1}else De?(at&&t.texStorage2D(n.TEXTURE_2D,te,Te,ee.width,ee.height),I&&t.texSubImage2D(n.TEXTURE_2D,0,0,0,ee.width,ee.height,ae,He,ee.data)):t.texImage2D(n.TEXTURE_2D,0,Te,ee.width,ee.height,0,ae,He,ee.data);else if(w.isCompressedTexture)if(w.isCompressedArrayTexture){De&&at&&t.texStorage3D(n.TEXTURE_2D_ARRAY,te,Te,Ae[0].width,Ae[0].height,ee.depth);for(let X=0,Y=Ae.length;X<Y;X++)if(de=Ae[X],w.format!==Un)if(ae!==null)if(De){if(I)if(w.layerUpdates.size>0){let ie=dv(de.width,de.height,w.format,w.type);for(let Se of w.layerUpdates){let Ge=de.data.subarray(Se*ie/de.data.BYTES_PER_ELEMENT,(Se+1)*ie/de.data.BYTES_PER_ELEMENT);t.compressedTexSubImage3D(n.TEXTURE_2D_ARRAY,X,0,0,Se,de.width,de.height,1,ae,Ge,0,0)}w.clearLayerUpdates()}else t.compressedTexSubImage3D(n.TEXTURE_2D_ARRAY,X,0,0,0,de.width,de.height,ee.depth,ae,de.data,0,0)}else t.compressedTexImage3D(n.TEXTURE_2D_ARRAY,X,Te,de.width,de.height,ee.depth,0,de.data,0,0);else console.warn("THREE.WebGLRenderer: Attempt to load unsupported compressed texture format in .uploadTexture()");else De?I&&t.texSubImage3D(n.TEXTURE_2D_ARRAY,X,0,0,0,de.width,de.height,ee.depth,ae,He,de.data):t.texImage3D(n.TEXTURE_2D_ARRAY,X,Te,de.width,de.height,ee.depth,0,ae,He,de.data)}else{De&&at&&t.texStorage2D(n.TEXTURE_2D,te,Te,Ae[0].width,Ae[0].height);for(let X=0,Y=Ae.length;X<Y;X++)de=Ae[X],w.format!==Un?ae!==null?De?I&&t.compressedTexSubImage2D(n.TEXTURE_2D,X,0,0,de.width,de.height,ae,de.data):t.compressedTexImage2D(n.TEXTURE_2D,X,Te,de.width,de.height,0,de.data):console.warn("THREE.WebGLRenderer: Attempt to load unsupported compressed texture format in .uploadTexture()"):De?I&&t.texSubImage2D(n.TEXTURE_2D,X,0,0,de.width,de.height,ae,He,de.data):t.texImage2D(n.TEXTURE_2D,X,Te,de.width,de.height,0,ae,He,de.data)}else if(w.isDataArrayTexture)if(De){if(at&&t.texStorage3D(n.TEXTURE_2D_ARRAY,te,Te,ee.width,ee.height,ee.depth),I)if(w.layerUpdates.size>0){let X=dv(ee.width,ee.height,w.format,w.type);for(let Y of w.layerUpdates){let ie=ee.data.subarray(Y*X/ee.data.BYTES_PER_ELEMENT,(Y+1)*X/ee.data.BYTES_PER_ELEMENT);t.texSubImage3D(n.TEXTURE_2D_ARRAY,0,0,0,Y,ee.width,ee.height,1,ae,He,ie)}w.clearLayerUpdates()}else t.texSubImage3D(n.TEXTURE_2D_ARRAY,0,0,0,0,ee.width,ee.height,ee.depth,ae,He,ee.data)}else t.texImage3D(n.TEXTURE_2D_ARRAY,0,Te,ee.width,ee.height,ee.depth,0,ae,He,ee.data);else if(w.isData3DTexture)De?(at&&t.texStorage3D(n.TEXTURE_3D,te,Te,ee.width,ee.height,ee.depth),I&&t.texSubImage3D(n.TEXTURE_3D,0,0,0,0,ee.width,ee.height,ee.depth,ae,He,ee.data)):t.texImage3D(n.TEXTURE_3D,0,Te,ee.width,ee.height,ee.depth,0,ae,He,ee.data);else if(w.isFramebufferTexture){if(at)if(De)t.texStorage2D(n.TEXTURE_2D,te,Te,ee.width,ee.height);else{let X=ee.width,Y=ee.height;for(let ie=0;ie<te;ie++)t.texImage2D(n.TEXTURE_2D,ie,Te,X,Y,0,ae,He,null),X>>=1,Y>>=1}}else if(Ae.length>0){if(De&&at){let X=Pe(Ae[0]);t.texStorage2D(n.TEXTURE_2D,te,Te,X.width,X.height)}for(let X=0,Y=Ae.length;X<Y;X++)de=Ae[X],De?I&&t.texSubImage2D(n.TEXTURE_2D,X,0,0,ae,He,de):t.texImage2D(n.TEXTURE_2D,X,Te,ae,He,de);w.generateMipmaps=!1}else if(De){if(at){let X=Pe(ee);t.texStorage2D(n.TEXTURE_2D,te,Te,X.width,X.height)}I&&t.texSubImage2D(n.TEXTURE_2D,0,0,0,ae,He,ee)}else t.texImage2D(n.TEXTURE_2D,0,Te,ae,He,ee);d(w)&&h(Z),xe.__version=q.version,w.onUpdate&&w.onUpdate(w)}b.__version=w.version}function H(b,w,B){if(w.image.length!==6)return;let Z=Ve(b,w),J=w.source;t.bindTexture(n.TEXTURE_CUBE_MAP,b.__webglTexture,n.TEXTURE0+B);let q=i.get(J);if(J.version!==q.__version||Z===!0){t.activeTexture(n.TEXTURE0+B);let xe=Ke.getPrimaries(Ke.workingColorSpace),se=w.colorSpace===Wi?null:Ke.getPrimaries(w.colorSpace),ue=w.colorSpace===Wi||xe===se?n.NONE:n.BROWSER_DEFAULT_WEBGL;n.pixelStorei(n.UNPACK_FLIP_Y_WEBGL,w.flipY),n.pixelStorei(n.UNPACK_PREMULTIPLY_ALPHA_WEBGL,w.premultiplyAlpha),n.pixelStorei(n.UNPACK_ALIGNMENT,w.unpackAlignment),n.pixelStorei(n.UNPACK_COLORSPACE_CONVERSION_WEBGL,ue);let Ie=w.isCompressedTexture||w.image[0].isCompressedTexture,ee=w.image[0]&&w.image[0].isDataTexture,ae=[];for(let Y=0;Y<6;Y++)!Ie&&!ee?ae[Y]=y(w.image[Y],!0,r.maxCubemapSize):ae[Y]=ee?w.image[Y].image:w.image[Y],ae[Y]=be(w,ae[Y]);let He=ae[0],Te=s.convert(w.format,w.colorSpace),de=s.convert(w.type),Ae=g(w.internalFormat,Te,de,w.colorSpace),De=w.isVideoTexture!==!0,at=q.__version===void 0||Z===!0,I=J.dataReady,te=x(w,He);_e(n.TEXTURE_CUBE_MAP,w);let X;if(Ie){De&&at&&t.texStorage2D(n.TEXTURE_CUBE_MAP,te,Ae,He.width,He.height);for(let Y=0;Y<6;Y++){X=ae[Y].mipmaps;for(let ie=0;ie<X.length;ie++){let Se=X[ie];w.format!==Un?Te!==null?De?I&&t.compressedTexSubImage2D(n.TEXTURE_CUBE_MAP_POSITIVE_X+Y,ie,0,0,Se.width,Se.height,Te,Se.data):t.compressedTexImage2D(n.TEXTURE_CUBE_MAP_POSITIVE_X+Y,ie,Ae,Se.width,Se.height,0,Se.data):console.warn("THREE.WebGLRenderer: Attempt to load unsupported compressed texture format in .setTextureCube()"):De?I&&t.texSubImage2D(n.TEXTURE_CUBE_MAP_POSITIVE_X+Y,ie,0,0,Se.width,Se.height,Te,de,Se.data):t.texImage2D(n.TEXTURE_CUBE_MAP_POSITIVE_X+Y,ie,Ae,Se.width,Se.height,0,Te,de,Se.data)}}}else{if(X=w.mipmaps,De&&at){X.length>0&&te++;let Y=Pe(ae[0]);t.texStorage2D(n.TEXTURE_CUBE_MAP,te,Ae,Y.width,Y.height)}for(let Y=0;Y<6;Y++)if(ee){De?I&&t.texSubImage2D(n.TEXTURE_CUBE_MAP_POSITIVE_X+Y,0,0,0,ae[Y].width,ae[Y].height,Te,de,ae[Y].data):t.texImage2D(n.TEXTURE_CUBE_MAP_POSITIVE_X+Y,0,Ae,ae[Y].width,ae[Y].height,0,Te,de,ae[Y].data);for(let ie=0;ie<X.length;ie++){let Ge=X[ie].image[Y].image;De?I&&t.texSubImage2D(n.TEXTURE_CUBE_MAP_POSITIVE_X+Y,ie+1,0,0,Ge.width,Ge.height,Te,de,Ge.data):t.texImage2D(n.TEXTURE_CUBE_MAP_POSITIVE_X+Y,ie+1,Ae,Ge.width,Ge.height,0,Te,de,Ge.data)}}else{De?I&&t.texSubImage2D(n.TEXTURE_CUBE_MAP_POSITIVE_X+Y,0,0,0,Te,de,ae[Y]):t.texImage2D(n.TEXTURE_CUBE_MAP_POSITIVE_X+Y,0,Ae,Te,de,ae[Y]);for(let ie=0;ie<X.length;ie++){let Se=X[ie];De?I&&t.texSubImage2D(n.TEXTURE_CUBE_MAP_POSITIVE_X+Y,ie+1,0,0,Te,de,Se.image[Y]):t.texImage2D(n.TEXTURE_CUBE_MAP_POSITIVE_X+Y,ie+1,Ae,Te,de,Se.image[Y])}}}d(w)&&h(n.TEXTURE_CUBE_MAP),q.__version=J.version,w.onUpdate&&w.onUpdate(w)}b.__version=w.version}function Q(b,w,B,Z,J,q){let xe=s.convert(B.format,B.colorSpace),se=s.convert(B.type),ue=g(B.internalFormat,xe,se,B.colorSpace);if(!i.get(w).__hasExternalTextures){let ee=Math.max(1,w.width>>q),ae=Math.max(1,w.height>>q);J===n.TEXTURE_3D||J===n.TEXTURE_2D_ARRAY?t.texImage3D(J,q,ue,ee,ae,w.depth,0,xe,se,null):t.texImage2D(J,q,ue,ee,ae,0,xe,se,null)}t.bindFramebuffer(n.FRAMEBUFFER,b),ye(w)?a.framebufferTexture2DMultisampleEXT(n.FRAMEBUFFER,Z,J,i.get(B).__webglTexture,0,Qe(w)):(J===n.TEXTURE_2D||J>=n.TEXTURE_CUBE_MAP_POSITIVE_X&&J<=n.TEXTURE_CUBE_MAP_NEGATIVE_Z)&&n.framebufferTexture2D(n.FRAMEBUFFER,Z,J,i.get(B).__webglTexture,q),t.bindFramebuffer(n.FRAMEBUFFER,null)}function fe(b,w,B){if(n.bindRenderbuffer(n.RENDERBUFFER,b),w.depthBuffer){let Z=w.depthTexture,J=Z&&Z.isDepthTexture?Z.type:null,q=v(w.stencilBuffer,J),xe=w.stencilBuffer?n.DEPTH_STENCIL_ATTACHMENT:n.DEPTH_ATTACHMENT,se=Qe(w);ye(w)?a.renderbufferStorageMultisampleEXT(n.RENDERBUFFER,se,q,w.width,w.height):B?n.renderbufferStorageMultisample(n.RENDERBUFFER,se,q,w.width,w.height):n.renderbufferStorage(n.RENDERBUFFER,q,w.width,w.height),n.framebufferRenderbuffer(n.FRAMEBUFFER,xe,n.RENDERBUFFER,b)}else{let Z=w.textures;for(let J=0;J<Z.length;J++){let q=Z[J],xe=s.convert(q.format,q.colorSpace),se=s.convert(q.type),ue=g(q.internalFormat,xe,se,q.colorSpace),Ie=Qe(w);B&&ye(w)===!1?n.renderbufferStorageMultisample(n.RENDERBUFFER,Ie,ue,w.width,w.height):ye(w)?a.renderbufferStorageMultisampleEXT(n.RENDERBUFFER,Ie,ue,w.width,w.height):n.renderbufferStorage(n.RENDERBUFFER,ue,w.width,w.height)}}n.bindRenderbuffer(n.RENDERBUFFER,null)}function ce(b,w){if(w&&w.isWebGLCubeRenderTarget)throw new Error("Depth Texture with cube render targets is not supported");if(t.bindFramebuffer(n.FRAMEBUFFER,b),!(w.depthTexture&&w.depthTexture.isDepthTexture))throw new Error("renderTarget.depthTexture must be an instance of THREE.DepthTexture");(!i.get(w.depthTexture).__webglTexture||w.depthTexture.image.width!==w.width||w.depthTexture.image.height!==w.height)&&(w.depthTexture.image.width=w.width,w.depthTexture.image.height=w.height,w.depthTexture.needsUpdate=!0),G(w.depthTexture,0);let Z=i.get(w.depthTexture).__webglTexture,J=Qe(w);if(w.depthTexture.format===Ns)ye(w)?a.framebufferTexture2DMultisampleEXT(n.FRAMEBUFFER,n.DEPTH_ATTACHMENT,n.TEXTURE_2D,Z,0,J):n.framebufferTexture2D(n.FRAMEBUFFER,n.DEPTH_ATTACHMENT,n.TEXTURE_2D,Z,0);else if(w.depthTexture.format===zs)ye(w)?a.framebufferTexture2DMultisampleEXT(n.FRAMEBUFFER,n.DEPTH_STENCIL_ATTACHMENT,n.TEXTURE_2D,Z,0,J):n.framebufferTexture2D(n.FRAMEBUFFER,n.DEPTH_STENCIL_ATTACHMENT,n.TEXTURE_2D,Z,0);else throw new Error("Unknown depthTexture format")}function Re(b){let w=i.get(b),B=b.isWebGLCubeRenderTarget===!0;if(b.depthTexture&&!w.__autoAllocateDepthBuffer){if(B)throw new Error("target.depthTexture not supported in Cube render targets");ce(w.__webglFramebuffer,b)}else if(B){w.__webglDepthbuffer=[];for(let Z=0;Z<6;Z++)t.bindFramebuffer(n.FRAMEBUFFER,w.__webglFramebuffer[Z]),w.__webglDepthbuffer[Z]=n.createRenderbuffer(),fe(w.__webglDepthbuffer[Z],b,!1)}else t.bindFramebuffer(n.FRAMEBUFFER,w.__webglFramebuffer),w.__webglDepthbuffer=n.createRenderbuffer(),fe(w.__webglDepthbuffer,b,!1);t.bindFramebuffer(n.FRAMEBUFFER,null)}function Ne(b,w,B){let Z=i.get(b);w!==void 0&&Q(Z.__webglFramebuffer,b,b.texture,n.COLOR_ATTACHMENT0,n.TEXTURE_2D,0),B!==void 0&&Re(b)}function Be(b){let w=b.texture,B=i.get(b),Z=i.get(w);b.addEventListener("dispose",C);let J=b.textures,q=b.isWebGLCubeRenderTarget===!0,xe=J.length>1;if(xe||(Z.__webglTexture===void 0&&(Z.__webglTexture=n.createTexture()),Z.__version=w.version,o.memory.textures++),q){B.__webglFramebuffer=[];for(let se=0;se<6;se++)if(w.mipmaps&&w.mipmaps.length>0){B.__webglFramebuffer[se]=[];for(let ue=0;ue<w.mipmaps.length;ue++)B.__webglFramebuffer[se][ue]=n.createFramebuffer()}else B.__webglFramebuffer[se]=n.createFramebuffer()}else{if(w.mipmaps&&w.mipmaps.length>0){B.__webglFramebuffer=[];for(let se=0;se<w.mipmaps.length;se++)B.__webglFramebuffer[se]=n.createFramebuffer()}else B.__webglFramebuffer=n.createFramebuffer();if(xe)for(let se=0,ue=J.length;se<ue;se++){let Ie=i.get(J[se]);Ie.__webglTexture===void 0&&(Ie.__webglTexture=n.createTexture(),o.memory.textures++)}if(b.samples>0&&ye(b)===!1){B.__webglMultisampledFramebuffer=n.createFramebuffer(),B.__webglColorRenderbuffer=[],t.bindFramebuffer(n.FRAMEBUFFER,B.__webglMultisampledFramebuffer);for(let se=0;se<J.length;se++){let ue=J[se];B.__webglColorRenderbuffer[se]=n.createRenderbuffer(),n.bindRenderbuffer(n.RENDERBUFFER,B.__webglColorRenderbuffer[se]);let Ie=s.convert(ue.format,ue.colorSpace),ee=s.convert(ue.type),ae=g(ue.internalFormat,Ie,ee,ue.colorSpace,b.isXRRenderTarget===!0),He=Qe(b);n.renderbufferStorageMultisample(n.RENDERBUFFER,He,ae,b.width,b.height),n.framebufferRenderbuffer(n.FRAMEBUFFER,n.COLOR_ATTACHMENT0+se,n.RENDERBUFFER,B.__webglColorRenderbuffer[se])}n.bindRenderbuffer(n.RENDERBUFFER,null),b.depthBuffer&&(B.__webglDepthRenderbuffer=n.createRenderbuffer(),fe(B.__webglDepthRenderbuffer,b,!0)),t.bindFramebuffer(n.FRAMEBUFFER,null)}}if(q){t.bindTexture(n.TEXTURE_CUBE_MAP,Z.__webglTexture),_e(n.TEXTURE_CUBE_MAP,w);for(let se=0;se<6;se++)if(w.mipmaps&&w.mipmaps.length>0)for(let ue=0;ue<w.mipmaps.length;ue++)Q(B.__webglFramebuffer[se][ue],b,w,n.COLOR_ATTACHMENT0,n.TEXTURE_CUBE_MAP_POSITIVE_X+se,ue);else Q(B.__webglFramebuffer[se],b,w,n.COLOR_ATTACHMENT0,n.TEXTURE_CUBE_MAP_POSITIVE_X+se,0);d(w)&&h(n.TEXTURE_CUBE_MAP),t.unbindTexture()}else if(xe){for(let se=0,ue=J.length;se<ue;se++){let Ie=J[se],ee=i.get(Ie);t.bindTexture(n.TEXTURE_2D,ee.__webglTexture),_e(n.TEXTURE_2D,Ie),Q(B.__webglFramebuffer,b,Ie,n.COLOR_ATTACHMENT0+se,n.TEXTURE_2D,0),d(Ie)&&h(n.TEXTURE_2D)}t.unbindTexture()}else{let se=n.TEXTURE_2D;if((b.isWebGL3DRenderTarget||b.isWebGLArrayRenderTarget)&&(se=b.isWebGL3DRenderTarget?n.TEXTURE_3D:n.TEXTURE_2D_ARRAY),t.bindTexture(se,Z.__webglTexture),_e(se,w),w.mipmaps&&w.mipmaps.length>0)for(let ue=0;ue<w.mipmaps.length;ue++)Q(B.__webglFramebuffer[ue],b,w,n.COLOR_ATTACHMENT0,se,ue);else Q(B.__webglFramebuffer,b,w,n.COLOR_ATTACHMENT0,se,0);d(w)&&h(se),t.unbindTexture()}b.depthBuffer&&Re(b)}function lt(b){let w=b.textures;for(let B=0,Z=w.length;B<Z;B++){let J=w[B];if(d(J)){let q=b.isWebGLCubeRenderTarget?n.TEXTURE_CUBE_MAP:n.TEXTURE_2D,xe=i.get(J).__webglTexture;t.bindTexture(q,xe),h(q),t.unbindTexture()}}}let A=[],pt=[];function Je(b){if(b.samples>0){if(ye(b)===!1){let w=b.textures,B=b.width,Z=b.height,J=n.COLOR_BUFFER_BIT,q=b.stencilBuffer?n.DEPTH_STENCIL_ATTACHMENT:n.DEPTH_ATTACHMENT,xe=i.get(b),se=w.length>1;if(se)for(let ue=0;ue<w.length;ue++)t.bindFramebuffer(n.FRAMEBUFFER,xe.__webglMultisampledFramebuffer),n.framebufferRenderbuffer(n.FRAMEBUFFER,n.COLOR_ATTACHMENT0+ue,n.RENDERBUFFER,null),t.bindFramebuffer(n.FRAMEBUFFER,xe.__webglFramebuffer),n.framebufferTexture2D(n.DRAW_FRAMEBUFFER,n.COLOR_ATTACHMENT0+ue,n.TEXTURE_2D,null,0);t.bindFramebuffer(n.READ_FRAMEBUFFER,xe.__webglMultisampledFramebuffer),t.bindFramebuffer(n.DRAW_FRAMEBUFFER,xe.__webglFramebuffer);for(let ue=0;ue<w.length;ue++){if(b.resolveDepthBuffer&&(b.depthBuffer&&(J|=n.DEPTH_BUFFER_BIT),b.stencilBuffer&&b.resolveStencilBuffer&&(J|=n.STENCIL_BUFFER_BIT)),se){n.framebufferRenderbuffer(n.READ_FRAMEBUFFER,n.COLOR_ATTACHMENT0,n.RENDERBUFFER,xe.__webglColorRenderbuffer[ue]);let Ie=i.get(w[ue]).__webglTexture;n.framebufferTexture2D(n.DRAW_FRAMEBUFFER,n.COLOR_ATTACHMENT0,n.TEXTURE_2D,Ie,0)}n.blitFramebuffer(0,0,B,Z,0,0,B,Z,J,n.NEAREST),l===!0&&(A.length=0,pt.length=0,A.push(n.COLOR_ATTACHMENT0+ue),b.depthBuffer&&b.resolveDepthBuffer===!1&&(A.push(q),pt.push(q),n.invalidateFramebuffer(n.DRAW_FRAMEBUFFER,pt)),n.invalidateFramebuffer(n.READ_FRAMEBUFFER,A))}if(t.bindFramebuffer(n.READ_FRAMEBUFFER,null),t.bindFramebuffer(n.DRAW_FRAMEBUFFER,null),se)for(let ue=0;ue<w.length;ue++){t.bindFramebuffer(n.FRAMEBUFFER,xe.__webglMultisampledFramebuffer),n.framebufferRenderbuffer(n.FRAMEBUFFER,n.COLOR_ATTACHMENT0+ue,n.RENDERBUFFER,xe.__webglColorRenderbuffer[ue]);let Ie=i.get(w[ue]).__webglTexture;t.bindFramebuffer(n.FRAMEBUFFER,xe.__webglFramebuffer),n.framebufferTexture2D(n.DRAW_FRAMEBUFFER,n.COLOR_ATTACHMENT0+ue,n.TEXTURE_2D,Ie,0)}t.bindFramebuffer(n.DRAW_FRAMEBUFFER,xe.__webglMultisampledFramebuffer)}else if(b.depthBuffer&&b.resolveDepthBuffer===!1&&l){let w=b.stencilBuffer?n.DEPTH_STENCIL_ATTACHMENT:n.DEPTH_ATTACHMENT;n.invalidateFramebuffer(n.DRAW_FRAMEBUFFER,[w])}}}function Qe(b){return Math.min(r.maxSamples,b.samples)}function ye(b){let w=i.get(b);return b.samples>0&&e.has("WEBGL_multisampled_render_to_texture")===!0&&w.__useRenderToTexture!==!1}function mt(b){let w=o.render.frame;u.get(b)!==w&&(u.set(b,w),b.update())}function be(b,w){let B=b.colorSpace,Z=b.format,J=b.type;return b.isCompressedTexture===!0||b.isVideoTexture===!0||B!==Ji&&B!==Wi&&(Ke.getTransfer(B)===rt?(Z!==Un||J!==di)&&console.warn("THREE.WebGLTextures: sRGB encoded textures have to use RGBAFormat and UnsignedByteType."):console.error("THREE.WebGLTextures: Unsupported texture color space:",B)),w}function Pe(b){return typeof HTMLImageElement<"u"&&b instanceof HTMLImageElement?(c.width=b.naturalWidth||b.width,c.height=b.naturalHeight||b.height):typeof VideoFrame<"u"&&b instanceof VideoFrame?(c.width=b.displayWidth,c.height=b.displayHeight):(c.width=b.width,c.height=b.height),c}this.allocateTextureUnit=W,this.resetTextureUnits=P,this.setTexture2D=G,this.setTexture2DArray=$,this.setTexture3D=V,this.setTextureCube=K,this.rebindTextures=Ne,this.setupRenderTarget=Be,this.updateRenderTargetMipmap=lt,this.updateMultisampleRenderTarget=Je,this.setupDepthRenderbuffer=Re,this.setupFrameBufferTexture=Q,this.useMultisampledRTT=ye}function dT(n,e){function t(i,r=Wi){let s,o=Ke.getTransfer(r);if(i===di)return n.UNSIGNED_BYTE;if(i===mp)return n.UNSIGNED_SHORT_4_4_4_4;if(i===gp)return n.UNSIGNED_SHORT_5_5_5_1;if(i===xv)return n.UNSIGNED_INT_5_9_9_9_REV;if(i===vv)return n.BYTE;if(i===yv)return n.SHORT;if(i===ta)return n.UNSIGNED_SHORT;if(i===pp)return n.INT;if(i===Tr)return n.UNSIGNED_INT;if(i===ci)return n.FLOAT;if(i===aa)return n.HALF_FLOAT;if(i===wv)return n.ALPHA;if(i===Sv)return n.RGB;if(i===Un)return n.RGBA;if(i===Mv)return n.LUMINANCE;if(i===Ev)return n.LUMINANCE_ALPHA;if(i===Ns)return n.DEPTH_COMPONENT;if(i===zs)return n.DEPTH_STENCIL;if(i===Cv)return n.RED;if(i===_p)return n.RED_INTEGER;if(i===Tv)return n.RG;if(i===vp)return n.RG_INTEGER;if(i===yp)return n.RGBA_INTEGER;if(i===tc||i===nc||i===ic||i===rc)if(o===rt)if(s=e.get("WEBGL_compressed_texture_s3tc_srgb"),s!==null){if(i===tc)return s.COMPRESSED_SRGB_S3TC_DXT1_EXT;if(i===nc)return s.COMPRESSED_SRGB_ALPHA_S3TC_DXT1_EXT;if(i===ic)return s.COMPRESSED_SRGB_ALPHA_S3TC_DXT3_EXT;if(i===rc)return s.COMPRESSED_SRGB_ALPHA_S3TC_DXT5_EXT}else return null;else if(s=e.get("WEBGL_compressed_texture_s3tc"),s!==null){if(i===tc)return s.COMPRESSED_RGB_S3TC_DXT1_EXT;if(i===nc)return s.COMPRESSED_RGBA_S3TC_DXT1_EXT;if(i===ic)return s.COMPRESSED_RGBA_S3TC_DXT3_EXT;if(i===rc)return s.COMPRESSED_RGBA_S3TC_DXT5_EXT}else return null;if(i===lf||i===cf||i===uf||i===hf)if(s=e.get("WEBGL_compressed_texture_pvrtc"),s!==null){if(i===lf)return s.COMPRESSED_RGB_PVRTC_4BPPV1_IMG;if(i===cf)return s.COMPRESSED_RGB_PVRTC_2BPPV1_IMG;if(i===uf)return s.COMPRESSED_RGBA_PVRTC_4BPPV1_IMG;if(i===hf)return s.COMPRESSED_RGBA_PVRTC_2BPPV1_IMG}else return null;if(i===df||i===ff||i===pf)if(s=e.get("WEBGL_compressed_texture_etc"),s!==null){if(i===df||i===ff)return o===rt?s.COMPRESSED_SRGB8_ETC2:s.COMPRESSED_RGB8_ETC2;if(i===pf)return o===rt?s.COMPRESSED_SRGB8_ALPHA8_ETC2_EAC:s.COMPRESSED_RGBA8_ETC2_EAC}else return null;if(i===mf||i===gf||i===_f||i===vf||i===yf||i===xf||i===wf||i===Sf||i===Mf||i===Ef||i===Cf||i===Tf||i===bf||i===Af)if(s=e.get("WEBGL_compressed_texture_astc"),s!==null){if(i===mf)return o===rt?s.COMPRESSED_SRGB8_ALPHA8_ASTC_4x4_KHR:s.COMPRESSED_RGBA_ASTC_4x4_KHR;if(i===gf)return o===rt?s.COMPRESSED_SRGB8_ALPHA8_ASTC_5x4_KHR:s.COMPRESSED_RGBA_ASTC_5x4_KHR;if(i===_f)return o===rt?s.COMPRESSED_SRGB8_ALPHA8_ASTC_5x5_KHR:s.COMPRESSED_RGBA_ASTC_5x5_KHR;if(i===vf)return o===rt?s.COMPRESSED_SRGB8_ALPHA8_ASTC_6x5_KHR:s.COMPRESSED_RGBA_ASTC_6x5_KHR;if(i===yf)return o===rt?s.COMPRESSED_SRGB8_ALPHA8_ASTC_6x6_KHR:s.COMPRESSED_RGBA_ASTC_6x6_KHR;if(i===xf)return o===rt?s.COMPRESSED_SRGB8_ALPHA8_ASTC_8x5_KHR:s.COMPRESSED_RGBA_ASTC_8x5_KHR;if(i===wf)return o===rt?s.COMPRESSED_SRGB8_ALPHA8_ASTC_8x6_KHR:s.COMPRESSED_RGBA_ASTC_8x6_KHR;if(i===Sf)return o===rt?s.COMPRESSED_SRGB8_ALPHA8_ASTC_8x8_KHR:s.COMPRESSED_RGBA_ASTC_8x8_KHR;if(i===Mf)return o===rt?s.COMPRESSED_SRGB8_ALPHA8_ASTC_10x5_KHR:s.COMPRESSED_RGBA_ASTC_10x5_KHR;if(i===Ef)return o===rt?s.COMPRESSED_SRGB8_ALPHA8_ASTC_10x6_KHR:s.COMPRESSED_RGBA_ASTC_10x6_KHR;if(i===Cf)return o===rt?s.COMPRESSED_SRGB8_ALPHA8_ASTC_10x8_KHR:s.COMPRESSED_RGBA_ASTC_10x8_KHR;if(i===Tf)return o===rt?s.COMPRESSED_SRGB8_ALPHA8_ASTC_10x10_KHR:s.COMPRESSED_RGBA_ASTC_10x10_KHR;if(i===bf)return o===rt?s.COMPRESSED_SRGB8_ALPHA8_ASTC_12x10_KHR:s.COMPRESSED_RGBA_ASTC_12x10_KHR;if(i===Af)return o===rt?s.COMPRESSED_SRGB8_ALPHA8_ASTC_12x12_KHR:s.COMPRESSED_RGBA_ASTC_12x12_KHR}else return null;if(i===sc||i===Rf||i===Pf)if(s=e.get("EXT_texture_compression_bptc"),s!==null){if(i===sc)return o===rt?s.COMPRESSED_SRGB_ALPHA_BPTC_UNORM_EXT:s.COMPRESSED_RGBA_BPTC_UNORM_EXT;if(i===Rf)return s.COMPRESSED_RGB_BPTC_SIGNED_FLOAT_EXT;if(i===Pf)return s.COMPRESSED_RGB_BPTC_UNSIGNED_FLOAT_EXT}else return null;if(i===bv||i===If||i===Lf||i===Uf)if(s=e.get("EXT_texture_compression_rgtc"),s!==null){if(i===sc)return s.COMPRESSED_RED_RGTC1_EXT;if(i===If)return s.COMPRESSED_SIGNED_RED_RGTC1_EXT;if(i===Lf)return s.COMPRESSED_RED_GREEN_RGTC2_EXT;if(i===Uf)return s.COMPRESSED_SIGNED_RED_GREEN_RGTC2_EXT}else return null;return i===ks?n.UNSIGNED_INT_24_8:n[i]!==void 0?n[i]:null}return{convert:t}}var Jf=class extends Yt{constructor(e=[]){super(),this.isArrayCamera=!0,this.cameras=e}},Xi=class extends Vt{constructor(){super(),this.isGroup=!0,this.type="Group"}},fT={type:"move"},ea=class{constructor(){this._targetRay=null,this._grip=null,this._hand=null}getHandSpace(){return this._hand===null&&(this._hand=new Xi,this._hand.matrixAutoUpdate=!1,this._hand.visible=!1,this._hand.joints={},this._hand.inputState={pinching:!1}),this._hand}getTargetRaySpace(){return this._targetRay===null&&(this._targetRay=new Xi,this._targetRay.matrixAutoUpdate=!1,this._targetRay.visible=!1,this._targetRay.hasLinearVelocity=!1,this._targetRay.linearVelocity=new D,this._targetRay.hasAngularVelocity=!1,this._targetRay.angularVelocity=new D),this._targetRay}getGripSpace(){return this._grip===null&&(this._grip=new Xi,this._grip.matrixAutoUpdate=!1,this._grip.visible=!1,this._grip.hasLinearVelocity=!1,this._grip.linearVelocity=new D,this._grip.hasAngularVelocity=!1,this._grip.angularVelocity=new D),this._grip}dispatchEvent(e){return this._targetRay!==null&&this._targetRay.dispatchEvent(e),this._grip!==null&&this._grip.dispatchEvent(e),this._hand!==null&&this._hand.dispatchEvent(e),this}connect(e){if(e&&e.hand){let t=this._hand;if(t)for(let i of e.hand.values())this._getHandJoint(t,i)}return this.dispatchEvent({type:"connected",data:e}),this}disconnect(e){return this.dispatchEvent({type:"disconnected",data:e}),this._targetRay!==null&&(this._targetRay.visible=!1),this._grip!==null&&(this._grip.visible=!1),this._hand!==null&&(this._hand.visible=!1),this}update(e,t,i){let r=null,s=null,o=null,a=this._targetRay,l=this._grip,c=this._hand;if(e&&t.session.visibilityState!=="visible-blurred"){if(c&&e.hand){o=!0;for(let y of e.hand.values()){let d=t.getJointPose(y,i),h=this._getHandJoint(c,y);d!==null&&(h.matrix.fromArray(d.transform.matrix),h.matrix.decompose(h.position,h.rotation,h.scale),h.matrixWorldNeedsUpdate=!0,h.jointRadius=d.radius),h.visible=d!==null}let u=c.joints["index-finger-tip"],p=c.joints["thumb-tip"],f=u.position.distanceTo(p.position),m=.02,_=.005;c.inputState.pinching&&f>m+_?(c.inputState.pinching=!1,this.dispatchEvent({type:"pinchend",handedness:e.handedness,target:this})):!c.inputState.pinching&&f<=m-_&&(c.inputState.pinching=!0,this.dispatchEvent({type:"pinchstart",handedness:e.handedness,target:this}))}else l!==null&&e.gripSpace&&(s=t.getPose(e.gripSpace,i),s!==null&&(l.matrix.fromArray(s.transform.matrix),l.matrix.decompose(l.position,l.rotation,l.scale),l.matrixWorldNeedsUpdate=!0,s.linearVelocity?(l.hasLinearVelocity=!0,l.linearVelocity.copy(s.linearVelocity)):l.hasLinearVelocity=!1,s.angularVelocity?(l.hasAngularVelocity=!0,l.angularVelocity.copy(s.angularVelocity)):l.hasAngularVelocity=!1));a!==null&&(r=t.getPose(e.targetRaySpace,i),r===null&&s!==null&&(r=s),r!==null&&(a.matrix.fromArray(r.transform.matrix),a.matrix.decompose(a.position,a.rotation,a.scale),a.matrixWorldNeedsUpdate=!0,r.linearVelocity?(a.hasLinearVelocity=!0,a.linearVelocity.copy(r.linearVelocity)):a.hasLinearVelocity=!1,r.angularVelocity?(a.hasAngularVelocity=!0,a.angularVelocity.copy(r.angularVelocity)):a.hasAngularVelocity=!1,this.dispatchEvent(fT)))}return a!==null&&(a.visible=r!==null),l!==null&&(l.visible=s!==null),c!==null&&(c.visible=o!==null),this}_getHandJoint(e,t){if(e.joints[t.jointName]===void 0){let i=new Xi;i.matrixAutoUpdate=!1,i.visible=!1,e.joints[t.jointName]=i,e.add(i)}return e.joints[t.jointName]}},pT=`
void main() {

	gl_Position = vec4( position, 1.0 );

}`,mT=`
uniform sampler2DArray depthColor;
uniform float depthWidth;
uniform float depthHeight;

void main() {

	vec2 coord = vec2( gl_FragCoord.x / depthWidth, gl_FragCoord.y / depthHeight );

	if ( coord.x >= 1.0 ) {

		gl_FragDepth = texture( depthColor, vec3( coord.x - 1.0, coord.y, 1 ) ).r;

	} else {

		gl_FragDepth = texture( depthColor, vec3( coord.x, coord.y, 0 ) ).r;

	}

}`,Kf=class{constructor(){this.texture=null,this.mesh=null,this.depthNear=0,this.depthFar=0}init(e,t,i){if(this.texture===null){let r=new $t,s=e.properties.get(r);s.__webglTexture=t.texture,(t.depthNear!=i.depthNear||t.depthFar!=i.depthFar)&&(this.depthNear=t.depthNear,this.depthFar=t.depthFar),this.texture=r}}getMesh(e){if(this.texture!==null&&this.mesh===null){let t=e.cameras[0].viewport,i=new Mn({vertexShader:pT,fragmentShader:mT,uniforms:{depthColor:{value:this.texture},depthWidth:{value:t.z},depthHeight:{value:t.w}}});this.mesh=new St(new Yn(20,20),i)}return this.mesh}reset(){this.texture=null,this.mesh=null}getDepthTexture(){return this.texture}},jf=class extends Zi{constructor(e,t){super();let i=this,r=null,s=1,o=null,a="local-floor",l=1,c=null,u=null,p=null,f=null,m=null,_=null,y=new Kf,d=t.getContextAttributes(),h=null,g=null,v=[],x=[],R=new ze,C=null,T=new Yt;T.layers.enable(1),T.viewport=new wt;let L=new Yt;L.layers.enable(2),L.viewport=new wt;let E=[T,L],M=new Jf;M.layers.enable(1),M.layers.enable(2);let P=null,W=null;this.cameraAutoUpdate=!0,this.enabled=!1,this.isPresenting=!1,this.getController=function(H){let Q=v[H];return Q===void 0&&(Q=new ea,v[H]=Q),Q.getTargetRaySpace()},this.getControllerGrip=function(H){let Q=v[H];return Q===void 0&&(Q=new ea,v[H]=Q),Q.getGripSpace()},this.getHand=function(H){let Q=v[H];return Q===void 0&&(Q=new ea,v[H]=Q),Q.getHandSpace()};function k(H){let Q=x.indexOf(H.inputSource);if(Q===-1)return;let fe=v[Q];fe!==void 0&&(fe.update(H.inputSource,H.frame,c||o),fe.dispatchEvent({type:H.type,data:H.inputSource}))}function G(){r.removeEventListener("select",k),r.removeEventListener("selectstart",k),r.removeEventListener("selectend",k),r.removeEventListener("squeeze",k),r.removeEventListener("squeezestart",k),r.removeEventListener("squeezeend",k),r.removeEventListener("end",G),r.removeEventListener("inputsourceschange",$);for(let H=0;H<v.length;H++){let Q=x[H];Q!==null&&(x[H]=null,v[H].disconnect(Q))}P=null,W=null,y.reset(),e.setRenderTarget(h),m=null,f=null,p=null,r=null,g=null,je.stop(),i.isPresenting=!1,e.setPixelRatio(C),e.setSize(R.width,R.height,!1),i.dispatchEvent({type:"sessionend"})}this.setFramebufferScaleFactor=function(H){s=H,i.isPresenting===!0&&console.warn("THREE.WebXRManager: Cannot change framebuffer scale while presenting.")},this.setReferenceSpaceType=function(H){a=H,i.isPresenting===!0&&console.warn("THREE.WebXRManager: Cannot change reference space type while presenting.")},this.getReferenceSpace=function(){return c||o},this.setReferenceSpace=function(H){c=H},this.getBaseLayer=function(){return f!==null?f:m},this.getBinding=function(){return p},this.getFrame=function(){return _},this.getSession=function(){return r},this.setSession=async function(H){if(r=H,r!==null){if(h=e.getRenderTarget(),r.addEventListener("select",k),r.addEventListener("selectstart",k),r.addEventListener("selectend",k),r.addEventListener("squeeze",k),r.addEventListener("squeezestart",k),r.addEventListener("squeezeend",k),r.addEventListener("end",G),r.addEventListener("inputsourceschange",$),d.xrCompatible!==!0&&await t.makeXRCompatible(),C=e.getPixelRatio(),e.getSize(R),r.renderState.layers===void 0){let Q={antialias:d.antialias,alpha:!0,depth:d.depth,stencil:d.stencil,framebufferScaleFactor:s};m=new XRWebGLLayer(r,t,Q),r.updateRenderState({baseLayer:m}),e.setPixelRatio(1),e.setSize(m.framebufferWidth,m.framebufferHeight,!1),g=new fi(m.framebufferWidth,m.framebufferHeight,{format:Un,type:di,colorSpace:e.outputColorSpace,stencilBuffer:d.stencil})}else{let Q=null,fe=null,ce=null;d.depth&&(ce=d.stencil?t.DEPTH24_STENCIL8:t.DEPTH_COMPONENT24,Q=d.stencil?zs:Ns,fe=d.stencil?ks:Tr);let Re={colorFormat:t.RGBA8,depthFormat:ce,scaleFactor:s};p=new XRWebGLBinding(r,t),f=p.createProjectionLayer(Re),r.updateRenderState({layers:[f]}),e.setPixelRatio(1),e.setSize(f.textureWidth,f.textureHeight,!1),g=new fi(f.textureWidth,f.textureHeight,{format:Un,type:di,depthTexture:new wc(f.textureWidth,f.textureHeight,fe,void 0,void 0,void 0,void 0,void 0,void 0,Q),stencilBuffer:d.stencil,colorSpace:e.outputColorSpace,samples:d.antialias?4:0,resolveDepthBuffer:f.ignoreDepthValues===!1})}g.isXRRenderTarget=!0,this.setFoveation(l),c=null,o=await r.requestReferenceSpace(a),je.setContext(r),je.start(),i.isPresenting=!0,i.dispatchEvent({type:"sessionstart"})}},this.getEnvironmentBlendMode=function(){if(r!==null)return r.environmentBlendMode},this.getDepthTexture=function(){return y.getDepthTexture()};function $(H){for(let Q=0;Q<H.removed.length;Q++){let fe=H.removed[Q],ce=x.indexOf(fe);ce>=0&&(x[ce]=null,v[ce].disconnect(fe))}for(let Q=0;Q<H.added.length;Q++){let fe=H.added[Q],ce=x.indexOf(fe);if(ce===-1){for(let Ne=0;Ne<v.length;Ne++)if(Ne>=x.length){x.push(fe),ce=Ne;break}else if(x[Ne]===null){x[Ne]=fe,ce=Ne;break}if(ce===-1)break}let Re=v[ce];Re&&Re.connect(fe)}}let V=new D,K=new D;function z(H,Q,fe){V.setFromMatrixPosition(Q.matrixWorld),K.setFromMatrixPosition(fe.matrixWorld);let ce=V.distanceTo(K),Re=Q.projectionMatrix.elements,Ne=fe.projectionMatrix.elements,Be=Re[14]/(Re[10]-1),lt=Re[14]/(Re[10]+1),A=(Re[9]+1)/Re[5],pt=(Re[9]-1)/Re[5],Je=(Re[8]-1)/Re[0],Qe=(Ne[8]+1)/Ne[0],ye=Be*Je,mt=Be*Qe,be=ce/(-Je+Qe),Pe=be*-Je;Q.matrixWorld.decompose(H.position,H.quaternion,H.scale),H.translateX(Pe),H.translateZ(be),H.matrixWorld.compose(H.position,H.quaternion,H.scale),H.matrixWorldInverse.copy(H.matrixWorld).invert();let b=Be+be,w=lt+be,B=ye-Pe,Z=mt+(ce-Pe),J=A*lt/w*b,q=pt*lt/w*b;H.projectionMatrix.makePerspective(B,Z,J,q,b,w),H.projectionMatrixInverse.copy(H.projectionMatrix).invert()}function he(H,Q){Q===null?H.matrixWorld.copy(H.matrix):H.matrixWorld.multiplyMatrices(Q.matrixWorld,H.matrix),H.matrixWorldInverse.copy(H.matrixWorld).invert()}this.updateCamera=function(H){if(r===null)return;y.texture!==null&&(H.near=y.depthNear,H.far=y.depthFar),M.near=L.near=T.near=H.near,M.far=L.far=T.far=H.far,(P!==M.near||W!==M.far)&&(r.updateRenderState({depthNear:M.near,depthFar:M.far}),P=M.near,W=M.far,T.near=P,T.far=W,L.near=P,L.far=W,T.updateProjectionMatrix(),L.updateProjectionMatrix(),H.updateProjectionMatrix());let Q=H.parent,fe=M.cameras;he(M,Q);for(let ce=0;ce<fe.length;ce++)he(fe[ce],Q);fe.length===2?z(M,T,L):M.projectionMatrix.copy(T.projectionMatrix),ge(H,M,Q)};function ge(H,Q,fe){fe===null?H.matrix.copy(Q.matrixWorld):(H.matrix.copy(fe.matrixWorld),H.matrix.invert(),H.matrix.multiply(Q.matrixWorld)),H.matrix.decompose(H.position,H.quaternion,H.scale),H.updateMatrixWorld(!0),H.projectionMatrix.copy(Q.projectionMatrix),H.projectionMatrixInverse.copy(Q.projectionMatrixInverse),H.isPerspectiveCamera&&(H.fov=Df*2*Math.atan(1/H.projectionMatrix.elements[5]),H.zoom=1)}this.getCamera=function(){return M},this.getFoveation=function(){if(!(f===null&&m===null))return l},this.setFoveation=function(H){l=H,f!==null&&(f.fixedFoveation=H),m!==null&&m.fixedFoveation!==void 0&&(m.fixedFoveation=H)},this.hasDepthSensing=function(){return y.texture!==null},this.getDepthSensingMesh=function(){return y.getMesh(M)};let _e=null;function Ve(H,Q){if(u=Q.getViewerPose(c||o),_=Q,u!==null){let fe=u.views;m!==null&&(e.setRenderTargetFramebuffer(g,m.framebuffer),e.setRenderTarget(g));let ce=!1;fe.length!==M.cameras.length&&(M.cameras.length=0,ce=!0);for(let Ne=0;Ne<fe.length;Ne++){let Be=fe[Ne],lt=null;if(m!==null)lt=m.getViewport(Be);else{let pt=p.getViewSubImage(f,Be);lt=pt.viewport,Ne===0&&(e.setRenderTargetTextures(g,pt.colorTexture,f.ignoreDepthValues?void 0:pt.depthStencilTexture),e.setRenderTarget(g))}let A=E[Ne];A===void 0&&(A=new Yt,A.layers.enable(Ne),A.viewport=new wt,E[Ne]=A),A.matrix.fromArray(Be.transform.matrix),A.matrix.decompose(A.position,A.quaternion,A.scale),A.projectionMatrix.fromArray(Be.projectionMatrix),A.projectionMatrixInverse.copy(A.projectionMatrix).invert(),A.viewport.set(lt.x,lt.y,lt.width,lt.height),Ne===0&&(M.matrix.copy(A.matrix),M.matrix.decompose(M.position,M.quaternion,M.scale)),ce===!0&&M.cameras.push(A)}let Re=r.enabledFeatures;if(Re&&Re.includes("depth-sensing")){let Ne=p.getDepthInformation(fe[0]);Ne&&Ne.isValid&&Ne.texture&&y.init(e,Ne,r.renderState)}}for(let fe=0;fe<v.length;fe++){let ce=x[fe],Re=v[fe];ce!==null&&Re!==void 0&&Re.update(ce,Q,c||o)}_e&&_e(H,Q),Q.detectedPlanes&&i.dispatchEvent({type:"planesdetected",data:Q}),_=null}let je=new Nv;je.setAnimationLoop(Ve),this.setAnimationLoop=function(H){_e=H},this.dispose=function(){}}},xr=new Xn,gT=new et;function _T(n,e){function t(d,h){d.matrixAutoUpdate===!0&&d.updateMatrix(),h.value.copy(d.matrix)}function i(d,h){h.color.getRGB(d.fogColor.value,Uv(n)),h.isFog?(d.fogNear.value=h.near,d.fogFar.value=h.far):h.isFogExp2&&(d.fogDensity.value=h.density)}function r(d,h,g,v,x){h.isMeshBasicMaterial||h.isMeshLambertMaterial?s(d,h):h.isMeshToonMaterial?(s(d,h),p(d,h)):h.isMeshPhongMaterial?(s(d,h),u(d,h)):h.isMeshStandardMaterial?(s(d,h),f(d,h),h.isMeshPhysicalMaterial&&m(d,h,x)):h.isMeshMatcapMaterial?(s(d,h),_(d,h)):h.isMeshDepthMaterial?s(d,h):h.isMeshDistanceMaterial?(s(d,h),y(d,h)):h.isMeshNormalMaterial?s(d,h):h.isLineBasicMaterial?(o(d,h),h.isLineDashedMaterial&&a(d,h)):h.isPointsMaterial?l(d,h,g,v):h.isSpriteMaterial?c(d,h):h.isShadowMaterial?(d.color.value.copy(h.color),d.opacity.value=h.opacity):h.isShaderMaterial&&(h.uniformsNeedUpdate=!1)}function s(d,h){d.opacity.value=h.opacity,h.color&&d.diffuse.value.copy(h.color),h.emissive&&d.emissive.value.copy(h.emissive).multiplyScalar(h.emissiveIntensity),h.map&&(d.map.value=h.map,t(h.map,d.mapTransform)),h.alphaMap&&(d.alphaMap.value=h.alphaMap,t(h.alphaMap,d.alphaMapTransform)),h.bumpMap&&(d.bumpMap.value=h.bumpMap,t(h.bumpMap,d.bumpMapTransform),d.bumpScale.value=h.bumpScale,h.side===tn&&(d.bumpScale.value*=-1)),h.normalMap&&(d.normalMap.value=h.normalMap,t(h.normalMap,d.normalMapTransform),d.normalScale.value.copy(h.normalScale),h.side===tn&&d.normalScale.value.negate()),h.displacementMap&&(d.displacementMap.value=h.displacementMap,t(h.displacementMap,d.displacementMapTransform),d.displacementScale.value=h.displacementScale,d.displacementBias.value=h.displacementBias),h.emissiveMap&&(d.emissiveMap.value=h.emissiveMap,t(h.emissiveMap,d.emissiveMapTransform)),h.specularMap&&(d.specularMap.value=h.specularMap,t(h.specularMap,d.specularMapTransform)),h.alphaTest>0&&(d.alphaTest.value=h.alphaTest);let g=e.get(h),v=g.envMap,x=g.envMapRotation;v&&(d.envMap.value=v,xr.copy(x),xr.x*=-1,xr.y*=-1,xr.z*=-1,v.isCubeTexture&&v.isRenderTargetTexture===!1&&(xr.y*=-1,xr.z*=-1),d.envMapRotation.value.setFromMatrix4(gT.makeRotationFromEuler(xr)),d.flipEnvMap.value=v.isCubeTexture&&v.isRenderTargetTexture===!1?-1:1,d.reflectivity.value=h.reflectivity,d.ior.value=h.ior,d.refractionRatio.value=h.refractionRatio),h.lightMap&&(d.lightMap.value=h.lightMap,d.lightMapIntensity.value=h.lightMapIntensity,t(h.lightMap,d.lightMapTransform)),h.aoMap&&(d.aoMap.value=h.aoMap,d.aoMapIntensity.value=h.aoMapIntensity,t(h.aoMap,d.aoMapTransform))}function o(d,h){d.diffuse.value.copy(h.color),d.opacity.value=h.opacity,h.map&&(d.map.value=h.map,t(h.map,d.mapTransform))}function a(d,h){d.dashSize.value=h.dashSize,d.totalSize.value=h.dashSize+h.gapSize,d.scale.value=h.scale}function l(d,h,g,v){d.diffuse.value.copy(h.color),d.opacity.value=h.opacity,d.size.value=h.size*g,d.scale.value=v*.5,h.map&&(d.map.value=h.map,t(h.map,d.uvTransform)),h.alphaMap&&(d.alphaMap.value=h.alphaMap,t(h.alphaMap,d.alphaMapTransform)),h.alphaTest>0&&(d.alphaTest.value=h.alphaTest)}function c(d,h){d.diffuse.value.copy(h.color),d.opacity.value=h.opacity,d.rotation.value=h.rotation,h.map&&(d.map.value=h.map,t(h.map,d.mapTransform)),h.alphaMap&&(d.alphaMap.value=h.alphaMap,t(h.alphaMap,d.alphaMapTransform)),h.alphaTest>0&&(d.alphaTest.value=h.alphaTest)}function u(d,h){d.specular.value.copy(h.specular),d.shininess.value=Math.max(h.shininess,1e-4)}function p(d,h){h.gradientMap&&(d.gradientMap.value=h.gradientMap)}function f(d,h){d.metalness.value=h.metalness,h.metalnessMap&&(d.metalnessMap.value=h.metalnessMap,t(h.metalnessMap,d.metalnessMapTransform)),d.roughness.value=h.roughness,h.roughnessMap&&(d.roughnessMap.value=h.roughnessMap,t(h.roughnessMap,d.roughnessMapTransform)),h.envMap&&(d.envMapIntensity.value=h.envMapIntensity)}function m(d,h,g){d.ior.value=h.ior,h.sheen>0&&(d.sheenColor.value.copy(h.sheenColor).multiplyScalar(h.sheen),d.sheenRoughness.value=h.sheenRoughness,h.sheenColorMap&&(d.sheenColorMap.value=h.sheenColorMap,t(h.sheenColorMap,d.sheenColorMapTransform)),h.sheenRoughnessMap&&(d.sheenRoughnessMap.value=h.sheenRoughnessMap,t(h.sheenRoughnessMap,d.sheenRoughnessMapTransform))),h.clearcoat>0&&(d.clearcoat.value=h.clearcoat,d.clearcoatRoughness.value=h.clearcoatRoughness,h.clearcoatMap&&(d.clearcoatMap.value=h.clearcoatMap,t(h.clearcoatMap,d.clearcoatMapTransform)),h.clearcoatRoughnessMap&&(d.clearcoatRoughnessMap.value=h.clearcoatRoughnessMap,t(h.clearcoatRoughnessMap,d.clearcoatRoughnessMapTransform)),h.clearcoatNormalMap&&(d.clearcoatNormalMap.value=h.clearcoatNormalMap,t(h.clearcoatNormalMap,d.clearcoatNormalMapTransform),d.clearcoatNormalScale.value.copy(h.clearcoatNormalScale),h.side===tn&&d.clearcoatNormalScale.value.negate())),h.dispersion>0&&(d.dispersion.value=h.dispersion),h.iridescence>0&&(d.iridescence.value=h.iridescence,d.iridescenceIOR.value=h.iridescenceIOR,d.iridescenceThicknessMinimum.value=h.iridescenceThicknessRange[0],d.iridescenceThicknessMaximum.value=h.iridescenceThicknessRange[1],h.iridescenceMap&&(d.iridescenceMap.value=h.iridescenceMap,t(h.iridescenceMap,d.iridescenceMapTransform)),h.iridescenceThicknessMap&&(d.iridescenceThicknessMap.value=h.iridescenceThicknessMap,t(h.iridescenceThicknessMap,d.iridescenceThicknessMapTransform))),h.transmission>0&&(d.transmission.value=h.transmission,d.transmissionSamplerMap.value=g.texture,d.transmissionSamplerSize.value.set(g.width,g.height),h.transmissionMap&&(d.transmissionMap.value=h.transmissionMap,t(h.transmissionMap,d.transmissionMapTransform)),d.thickness.value=h.thickness,h.thicknessMap&&(d.thicknessMap.value=h.thicknessMap,t(h.thicknessMap,d.thicknessMapTransform)),d.attenuationDistance.value=h.attenuationDistance,d.attenuationColor.value.copy(h.attenuationColor)),h.anisotropy>0&&(d.anisotropyVector.value.set(h.anisotropy*Math.cos(h.anisotropyRotation),h.anisotropy*Math.sin(h.anisotropyRotation)),h.anisotropyMap&&(d.anisotropyMap.value=h.anisotropyMap,t(h.anisotropyMap,d.anisotropyMapTransform))),d.specularIntensity.value=h.specularIntensity,d.specularColor.value.copy(h.specularColor),h.specularColorMap&&(d.specularColorMap.value=h.specularColorMap,t(h.specularColorMap,d.specularColorMapTransform)),h.specularIntensityMap&&(d.specularIntensityMap.value=h.specularIntensityMap,t(h.specularIntensityMap,d.specularIntensityMapTransform))}function _(d,h){h.matcap&&(d.matcap.value=h.matcap)}function y(d,h){let g=e.get(h).light;d.referencePosition.value.setFromMatrixPosition(g.matrixWorld),d.nearDistance.value=g.shadow.camera.near,d.farDistance.value=g.shadow.camera.far}return{refreshFogUniforms:i,refreshMaterialUniforms:r}}function vT(n,e,t,i){let r={},s={},o=[],a=n.getParameter(n.MAX_UNIFORM_BUFFER_BINDINGS);function l(g,v){let x=v.program;i.uniformBlockBinding(g,x)}function c(g,v){let x=r[g.id];x===void 0&&(_(g),x=u(g),r[g.id]=x,g.addEventListener("dispose",d));let R=v.program;i.updateUBOMapping(g,R);let C=e.render.frame;s[g.id]!==C&&(f(g),s[g.id]=C)}function u(g){let v=p();g.__bindingPointIndex=v;let x=n.createBuffer(),R=g.__size,C=g.usage;return n.bindBuffer(n.UNIFORM_BUFFER,x),n.bufferData(n.UNIFORM_BUFFER,R,C),n.bindBuffer(n.UNIFORM_BUFFER,null),n.bindBufferBase(n.UNIFORM_BUFFER,v,x),x}function p(){for(let g=0;g<a;g++)if(o.indexOf(g)===-1)return o.push(g),g;return console.error("THREE.WebGLRenderer: Maximum number of simultaneously usable uniforms groups reached."),0}function f(g){let v=r[g.id],x=g.uniforms,R=g.__cache;n.bindBuffer(n.UNIFORM_BUFFER,v);for(let C=0,T=x.length;C<T;C++){let L=Array.isArray(x[C])?x[C]:[x[C]];for(let E=0,M=L.length;E<M;E++){let P=L[E];if(m(P,C,E,R)===!0){let W=P.__offset,k=Array.isArray(P.value)?P.value:[P.value],G=0;for(let $=0;$<k.length;$++){let V=k[$],K=y(V);typeof V=="number"||typeof V=="boolean"?(P.__data[0]=V,n.bufferSubData(n.UNIFORM_BUFFER,W+G,P.__data)):V.isMatrix3?(P.__data[0]=V.elements[0],P.__data[1]=V.elements[1],P.__data[2]=V.elements[2],P.__data[3]=0,P.__data[4]=V.elements[3],P.__data[5]=V.elements[4],P.__data[6]=V.elements[5],P.__data[7]=0,P.__data[8]=V.elements[6],P.__data[9]=V.elements[7],P.__data[10]=V.elements[8],P.__data[11]=0):(V.toArray(P.__data,G),G+=K.storage/Float32Array.BYTES_PER_ELEMENT)}n.bufferSubData(n.UNIFORM_BUFFER,W,P.__data)}}}n.bindBuffer(n.UNIFORM_BUFFER,null)}function m(g,v,x,R){let C=g.value,T=v+"_"+x;if(R[T]===void 0)return typeof C=="number"||typeof C=="boolean"?R[T]=C:R[T]=C.clone(),!0;{let L=R[T];if(typeof C=="number"||typeof C=="boolean"){if(L!==C)return R[T]=C,!0}else if(L.equals(C)===!1)return L.copy(C),!0}return!1}function _(g){let v=g.uniforms,x=0,R=16;for(let T=0,L=v.length;T<L;T++){let E=Array.isArray(v[T])?v[T]:[v[T]];for(let M=0,P=E.length;M<P;M++){let W=E[M],k=Array.isArray(W.value)?W.value:[W.value];for(let G=0,$=k.length;G<$;G++){let V=k[G],K=y(V),z=x%R;z!==0&&R-z<K.boundary&&(x+=R-z),W.__data=new Float32Array(K.storage/Float32Array.BYTES_PER_ELEMENT),W.__offset=x,x+=K.storage}}}let C=x%R;return C>0&&(x+=R-C),g.__size=x,g.__cache={},this}function y(g){let v={boundary:0,storage:0};return typeof g=="number"||typeof g=="boolean"?(v.boundary=4,v.storage=4):g.isVector2?(v.boundary=8,v.storage=8):g.isVector3||g.isColor?(v.boundary=16,v.storage=12):g.isVector4?(v.boundary=16,v.storage=16):g.isMatrix3?(v.boundary=48,v.storage=48):g.isMatrix4?(v.boundary=64,v.storage=64):g.isTexture?console.warn("THREE.WebGLRenderer: Texture samplers can not be part of an uniforms group."):console.warn("THREE.WebGLRenderer: Unsupported uniform value type.",g),v}function d(g){let v=g.target;v.removeEventListener("dispose",d);let x=o.indexOf(v.__bindingPointIndex);o.splice(x,1),n.deleteBuffer(r[v.id]),delete r[v.id],delete s[v.id]}function h(){for(let g in r)n.deleteBuffer(r[g]);o=[],r={},s={}}return{bind:l,update:c,dispose:h}}var Sc=class{constructor(e={}){let{canvas:t=CS(),context:i=null,depth:r=!0,stencil:s=!1,alpha:o=!1,antialias:a=!1,premultipliedAlpha:l=!0,preserveDrawingBuffer:c=!1,powerPreference:u="default",failIfMajorPerformanceCaveat:p=!1}=e;this.isWebGLRenderer=!0;let f;if(i!==null){if(typeof WebGLRenderingContext<"u"&&i instanceof WebGLRenderingContext)throw new Error("THREE.WebGLRenderer: WebGL 1 is not supported since r163.");f=i.getContextAttributes().alpha}else f=o;let m=new Uint32Array(4),_=new Int32Array(4),y=null,d=null,h=[],g=[];this.domElement=t,this.debug={checkShaderErrors:!0,onShaderError:null},this.autoClear=!0,this.autoClearColor=!0,this.autoClearDepth=!0,this.autoClearStencil=!0,this.sortObjects=!0,this.clippingPlanes=[],this.localClippingEnabled=!1,this._outputColorSpace=un,this.toneMapping=qi,this.toneMappingExposure=1;let v=this,x=!1,R=0,C=0,T=null,L=-1,E=null,M=new wt,P=new wt,W=null,k=new Oe(0),G=0,$=t.width,V=t.height,K=1,z=null,he=null,ge=new wt(0,0,$,V),_e=new wt(0,0,$,V),Ve=!1,je=new ra,H=!1,Q=!1,fe=new et,ce=new D,Re=new wt,Ne={background:null,fog:null,environment:null,overrideMaterial:null,isScene:!0},Be=!1;function lt(){return T===null?K:1}let A=i;function pt(S,U){return t.getContext(S,U)}try{let S={alpha:!0,depth:r,stencil:s,antialias:a,premultipliedAlpha:l,preserveDrawingBuffer:c,powerPreference:u,failIfMajorPerformanceCaveat:p};if("setAttribute"in t&&t.setAttribute("data-engine",`three.js r${dp}`),t.addEventListener("webglcontextlost",X,!1),t.addEventListener("webglcontextrestored",Y,!1),t.addEventListener("webglcontextcreationerror",ie,!1),A===null){let U="webgl2";if(A=pt(U,S),A===null)throw pt(U)?new Error("Error creating WebGL context with your selected attributes."):new Error("Error creating WebGL context.")}}catch(S){throw console.error("THREE.WebGLRenderer: "+S.message),S}let Je,Qe,ye,mt,be,Pe,b,w,B,Z,J,q,xe,se,ue,Ie,ee,ae,He,Te,de,Ae,De,at;function I(){Je=new OE(A),Je.init(),Ae=new dT(A,Je),Qe=new PE(A,Je,e,Ae),ye=new cT(A),mt=new kE(A),be=new KC,Pe=new hT(A,Je,ye,be,Qe,Ae,mt),b=new LE(v),w=new DE(v),B=new YS(A),De=new AE(A,B),Z=new FE(A,B,mt,De),J=new VE(A,Z,B,mt),He=new zE(A,Qe,Pe),Ie=new IE(be),q=new JC(v,b,w,Je,Qe,De,Ie),xe=new _T(v,be),se=new QC,ue=new sT(Je),ae=new bE(v,b,w,ye,J,f,l),ee=new lT(v,J,Qe),at=new vT(A,mt,Qe,ye),Te=new RE(A,Je,mt),de=new BE(A,Je,mt),mt.programs=q.programs,v.capabilities=Qe,v.extensions=Je,v.properties=be,v.renderLists=se,v.shadowMap=ee,v.state=ye,v.info=mt}I();let te=new jf(v,A);this.xr=te,this.getContext=function(){return A},this.getContextAttributes=function(){return A.getContextAttributes()},this.forceContextLoss=function(){let S=Je.get("WEBGL_lose_context");S&&S.loseContext()},this.forceContextRestore=function(){let S=Je.get("WEBGL_lose_context");S&&S.restoreContext()},this.getPixelRatio=function(){return K},this.setPixelRatio=function(S){S!==void 0&&(K=S,this.setSize($,V,!1))},this.getSize=function(S){return S.set($,V)},this.setSize=function(S,U,O=!0){if(te.isPresenting){console.warn("THREE.WebGLRenderer: Can't change size while VR device is presenting.");return}$=S,V=U,t.width=Math.floor(S*K),t.height=Math.floor(U*K),O===!0&&(t.style.width=S+"px",t.style.height=U+"px"),this.setViewport(0,0,S,U)},this.getDrawingBufferSize=function(S){return S.set($*K,V*K).floor()},this.setDrawingBufferSize=function(S,U,O){$=S,V=U,K=O,t.width=Math.floor(S*O),t.height=Math.floor(U*O),this.setViewport(0,0,S,U)},this.getCurrentViewport=function(S){return S.copy(M)},this.getViewport=function(S){return S.copy(ge)},this.setViewport=function(S,U,O,F){S.isVector4?ge.set(S.x,S.y,S.z,S.w):ge.set(S,U,O,F),ye.viewport(M.copy(ge).multiplyScalar(K).round())},this.getScissor=function(S){return S.copy(_e)},this.setScissor=function(S,U,O,F){S.isVector4?_e.set(S.x,S.y,S.z,S.w):_e.set(S,U,O,F),ye.scissor(P.copy(_e).multiplyScalar(K).round())},this.getScissorTest=function(){return Ve},this.setScissorTest=function(S){ye.setScissorTest(Ve=S)},this.setOpaqueSort=function(S){z=S},this.setTransparentSort=function(S){he=S},this.getClearColor=function(S){return S.copy(ae.getClearColor())},this.setClearColor=function(){ae.setClearColor.apply(ae,arguments)},this.getClearAlpha=function(){return ae.getClearAlpha()},this.setClearAlpha=function(){ae.setClearAlpha.apply(ae,arguments)},this.clear=function(S=!0,U=!0,O=!0){let F=0;if(S){let N=!1;if(T!==null){let ne=T.texture.format;N=ne===yp||ne===vp||ne===_p}if(N){let ne=T.texture.type,oe=ne===di||ne===Tr||ne===ta||ne===ks||ne===mp||ne===gp,pe=ae.getClearColor(),me=ae.getClearAlpha(),Me=pe.r,Ee=pe.g,we=pe.b;oe?(m[0]=Me,m[1]=Ee,m[2]=we,m[3]=me,A.clearBufferuiv(A.COLOR,0,m)):(_[0]=Me,_[1]=Ee,_[2]=we,_[3]=me,A.clearBufferiv(A.COLOR,0,_))}else F|=A.COLOR_BUFFER_BIT}U&&(F|=A.DEPTH_BUFFER_BIT),O&&(F|=A.STENCIL_BUFFER_BIT,this.state.buffers.stencil.setMask(4294967295)),A.clear(F)},this.clearColor=function(){this.clear(!0,!1,!1)},this.clearDepth=function(){this.clear(!1,!0,!1)},this.clearStencil=function(){this.clear(!1,!1,!0)},this.dispose=function(){t.removeEventListener("webglcontextlost",X,!1),t.removeEventListener("webglcontextrestored",Y,!1),t.removeEventListener("webglcontextcreationerror",ie,!1),se.dispose(),ue.dispose(),be.dispose(),b.dispose(),w.dispose(),J.dispose(),De.dispose(),at.dispose(),q.dispose(),te.dispose(),te.removeEventListener("sessionstart",On),te.removeEventListener("sessionend",Wp),ji.stop()};function X(S){S.preventDefault(),console.log("THREE.WebGLRenderer: Context Lost."),x=!0}function Y(){console.log("THREE.WebGLRenderer: Context Restored."),x=!1;let S=mt.autoReset,U=ee.enabled,O=ee.autoUpdate,F=ee.needsUpdate,N=ee.type;I(),mt.autoReset=S,ee.enabled=U,ee.autoUpdate=O,ee.needsUpdate=F,ee.type=N}function ie(S){console.error("THREE.WebGLRenderer: A WebGL context could not be created. Reason: ",S.statusMessage)}function Se(S){let U=S.target;U.removeEventListener("dispose",Se),Ge(U)}function Ge(S){gt(S),be.remove(S)}function gt(S){let U=be.get(S).programs;U!==void 0&&(U.forEach(function(O){q.releaseProgram(O)}),S.isShaderMaterial&&q.releaseShaderCache(S))}this.renderBufferDirect=function(S,U,O,F,N,ne){U===null&&(U=Ne);let oe=N.isMesh&&N.matrixWorld.determinant()<0,pe=gy(S,U,O,F,N);ye.setMaterial(F,oe);let me=O.index,Me=1;if(F.wireframe===!0){if(me=Z.getWireframeAttribute(O),me===void 0)return;Me=2}let Ee=O.drawRange,we=O.attributes.position,Ye=Ee.start*Me,ct=(Ee.start+Ee.count)*Me;ne!==null&&(Ye=Math.max(Ye,ne.start*Me),ct=Math.min(ct,(ne.start+ne.count)*Me)),me!==null?(Ye=Math.max(Ye,0),ct=Math.min(ct,me.count)):we!=null&&(Ye=Math.max(Ye,0),ct=Math.min(ct,we.count));let ut=ct-Ye;if(ut<0||ut===1/0)return;De.setup(N,F,pe,O,me);let rn,qe=Te;if(me!==null&&(rn=B.get(me),qe=de,qe.setIndex(rn)),N.isMesh)F.wireframe===!0?(ye.setLineWidth(F.wireframeLinewidth*lt()),qe.setMode(A.LINES)):qe.setMode(A.TRIANGLES);else if(N.isLine){let ve=F.linewidth;ve===void 0&&(ve=1),ye.setLineWidth(ve*lt()),N.isLineSegments?qe.setMode(A.LINES):N.isLineLoop?qe.setMode(A.LINE_LOOP):qe.setMode(A.LINE_STRIP)}else N.isPoints?qe.setMode(A.POINTS):N.isSprite&&qe.setMode(A.TRIANGLES);if(N.isBatchedMesh)if(N._multiDrawInstances!==null)qe.renderMultiDrawInstances(N._multiDrawStarts,N._multiDrawCounts,N._multiDrawCount,N._multiDrawInstances);else if(Je.get("WEBGL_multi_draw"))qe.renderMultiDraw(N._multiDrawStarts,N._multiDrawCounts,N._multiDrawCount);else{let ve=N._multiDrawStarts,At=N._multiDrawCounts,$e=N._multiDrawCount,Cn=me?B.get(me).bytesPerElement:1,Nr=be.get(F).currentProgram.getUniforms();for(let sn=0;sn<$e;sn++)Nr.setValue(A,"_gl_DrawID",sn),qe.render(ve[sn]/Cn,At[sn])}else if(N.isInstancedMesh)qe.renderInstances(Ye,ut,N.count);else if(O.isInstancedBufferGeometry){let ve=O._maxInstanceCount!==void 0?O._maxInstanceCount:1/0,At=Math.min(O.instanceCount,ve);qe.renderInstances(Ye,ut,At)}else qe.render(Ye,ut)};function bt(S,U,O){S.transparent===!0&&S.side===li&&S.forceSinglePass===!1?(S.side=tn,S.needsUpdate=!0,_a(S,U,O),S.side=$i,S.needsUpdate=!0,_a(S,U,O),S.side=li):_a(S,U,O)}this.compile=function(S,U,O=null){O===null&&(O=S),d=ue.get(O),d.init(U),g.push(d),O.traverseVisible(function(N){N.isLight&&N.layers.test(U.layers)&&(d.pushLight(N),N.castShadow&&d.pushShadow(N))}),S!==O&&S.traverseVisible(function(N){N.isLight&&N.layers.test(U.layers)&&(d.pushLight(N),N.castShadow&&d.pushShadow(N))}),d.setupLights();let F=new Set;return S.traverse(function(N){let ne=N.material;if(ne)if(Array.isArray(ne))for(let oe=0;oe<ne.length;oe++){let pe=ne[oe];bt(pe,O,N),F.add(pe)}else bt(ne,O,N),F.add(ne)}),g.pop(),d=null,F},this.compileAsync=function(S,U,O=null){let F=this.compile(S,U,O);return new Promise(N=>{function ne(){if(F.forEach(function(oe){be.get(oe).currentProgram.isReady()&&F.delete(oe)}),F.size===0){N(S);return}setTimeout(ne,10)}Je.get("KHR_parallel_shader_compile")!==null?ne():setTimeout(ne,10)})};let Xe=null;function qn(S){Xe&&Xe(S)}function On(){ji.stop()}function Wp(){ji.start()}let ji=new Nv;ji.setAnimationLoop(qn),typeof self<"u"&&ji.setContext(self),this.setAnimationLoop=function(S){Xe=S,te.setAnimationLoop(S),S===null?ji.stop():ji.start()},te.addEventListener("sessionstart",On),te.addEventListener("sessionend",Wp),this.render=function(S,U){if(U!==void 0&&U.isCamera!==!0){console.error("THREE.WebGLRenderer.render: camera is not an instance of THREE.Camera.");return}if(x===!0)return;if(S.matrixWorldAutoUpdate===!0&&S.updateMatrixWorld(),U.parent===null&&U.matrixWorldAutoUpdate===!0&&U.updateMatrixWorld(),te.enabled===!0&&te.isPresenting===!0&&(te.cameraAutoUpdate===!0&&te.updateCamera(U),U=te.getCamera()),S.isScene===!0&&S.onBeforeRender(v,S,U,T),d=ue.get(S,g.length),d.init(U),g.push(d),fe.multiplyMatrices(U.projectionMatrix,U.matrixWorldInverse),je.setFromProjectionMatrix(fe),Q=this.localClippingEnabled,H=Ie.init(this.clippingPlanes,Q),y=se.get(S,h.length),y.init(),h.push(y),te.enabled===!0&&te.isPresenting===!0){let ne=v.xr.getDepthSensingMesh();ne!==null&&ru(ne,U,-1/0,v.sortObjects)}ru(S,U,0,v.sortObjects),y.finish(),v.sortObjects===!0&&y.sort(z,he),Be=te.enabled===!1||te.isPresenting===!1||te.hasDepthSensing()===!1,Be&&ae.addToRenderList(y,S),this.info.render.frame++,H===!0&&Ie.beginShadows();let O=d.state.shadowsArray;ee.render(O,S,U),H===!0&&Ie.endShadows(),this.info.autoReset===!0&&this.info.reset();let F=y.opaque,N=y.transmissive;if(d.setupLights(),U.isArrayCamera){let ne=U.cameras;if(N.length>0)for(let oe=0,pe=ne.length;oe<pe;oe++){let me=ne[oe];Yp(F,N,S,me)}Be&&ae.render(S);for(let oe=0,pe=ne.length;oe<pe;oe++){let me=ne[oe];Xp(y,S,me,me.viewport)}}else N.length>0&&Yp(F,N,S,U),Be&&ae.render(S),Xp(y,S,U);T!==null&&(Pe.updateMultisampleRenderTarget(T),Pe.updateRenderTargetMipmap(T)),S.isScene===!0&&S.onAfterRender(v,S,U),De.resetDefaultState(),L=-1,E=null,g.pop(),g.length>0?(d=g[g.length-1],H===!0&&Ie.setGlobalState(v.clippingPlanes,d.state.camera)):d=null,h.pop(),h.length>0?y=h[h.length-1]:y=null};function ru(S,U,O,F){if(S.visible===!1)return;if(S.layers.test(U.layers)){if(S.isGroup)O=S.renderOrder;else if(S.isLOD)S.autoUpdate===!0&&S.update(U);else if(S.isLight)d.pushLight(S),S.castShadow&&d.pushShadow(S);else if(S.isSprite){if(!S.frustumCulled||je.intersectsSprite(S)){F&&Re.setFromMatrixPosition(S.matrixWorld).applyMatrix4(fe);let oe=J.update(S),pe=S.material;pe.visible&&y.push(S,oe,pe,O,Re.z,null)}}else if((S.isMesh||S.isLine||S.isPoints)&&(!S.frustumCulled||je.intersectsObject(S))){let oe=J.update(S),pe=S.material;if(F&&(S.boundingSphere!==void 0?(S.boundingSphere===null&&S.computeBoundingSphere(),Re.copy(S.boundingSphere.center)):(oe.boundingSphere===null&&oe.computeBoundingSphere(),Re.copy(oe.boundingSphere.center)),Re.applyMatrix4(S.matrixWorld).applyMatrix4(fe)),Array.isArray(pe)){let me=oe.groups;for(let Me=0,Ee=me.length;Me<Ee;Me++){let we=me[Me],Ye=pe[we.materialIndex];Ye&&Ye.visible&&y.push(S,oe,Ye,O,Re.z,we)}}else pe.visible&&y.push(S,oe,pe,O,Re.z,null)}}let ne=S.children;for(let oe=0,pe=ne.length;oe<pe;oe++)ru(ne[oe],U,O,F)}function Xp(S,U,O,F){let N=S.opaque,ne=S.transmissive,oe=S.transparent;d.setupLightsView(O),H===!0&&Ie.setGlobalState(v.clippingPlanes,O),F&&ye.viewport(M.copy(F)),N.length>0&&ga(N,U,O),ne.length>0&&ga(ne,U,O),oe.length>0&&ga(oe,U,O),ye.buffers.depth.setTest(!0),ye.buffers.depth.setMask(!0),ye.buffers.color.setMask(!0),ye.setPolygonOffset(!1)}function Yp(S,U,O,F){if((O.isScene===!0?O.overrideMaterial:null)!==null)return;d.state.transmissionRenderTarget[F.id]===void 0&&(d.state.transmissionRenderTarget[F.id]=new fi(1,1,{generateMipmaps:!0,type:Je.has("EXT_color_buffer_half_float")||Je.has("EXT_color_buffer_float")?aa:di,minFilter:Cr,samples:4,stencilBuffer:s,resolveDepthBuffer:!1,resolveStencilBuffer:!1,colorSpace:Ke.workingColorSpace}));let ne=d.state.transmissionRenderTarget[F.id],oe=F.viewport||M;ne.setSize(oe.z,oe.w);let pe=v.getRenderTarget();v.setRenderTarget(ne),v.getClearColor(k),G=v.getClearAlpha(),G<1&&v.setClearColor(16777215,.5),Be?ae.render(O):v.clear();let me=v.toneMapping;v.toneMapping=qi;let Me=F.viewport;if(F.viewport!==void 0&&(F.viewport=void 0),d.setupLightsView(F),H===!0&&Ie.setGlobalState(v.clippingPlanes,F),ga(S,O,F),Pe.updateMultisampleRenderTarget(ne),Pe.updateRenderTargetMipmap(ne),Je.has("WEBGL_multisampled_render_to_texture")===!1){let Ee=!1;for(let we=0,Ye=U.length;we<Ye;we++){let ct=U[we],ut=ct.object,rn=ct.geometry,qe=ct.material,ve=ct.group;if(qe.side===li&&ut.layers.test(F.layers)){let At=qe.side;qe.side=tn,qe.needsUpdate=!0,qp(ut,O,F,rn,qe,ve),qe.side=At,qe.needsUpdate=!0,Ee=!0}}Ee===!0&&(Pe.updateMultisampleRenderTarget(ne),Pe.updateRenderTargetMipmap(ne))}v.setRenderTarget(pe),v.setClearColor(k,G),Me!==void 0&&(F.viewport=Me),v.toneMapping=me}function ga(S,U,O){let F=U.isScene===!0?U.overrideMaterial:null;for(let N=0,ne=S.length;N<ne;N++){let oe=S[N],pe=oe.object,me=oe.geometry,Me=F===null?oe.material:F,Ee=oe.group;pe.layers.test(O.layers)&&qp(pe,U,O,me,Me,Ee)}}function qp(S,U,O,F,N,ne){S.onBeforeRender(v,U,O,F,N,ne),S.modelViewMatrix.multiplyMatrices(O.matrixWorldInverse,S.matrixWorld),S.normalMatrix.getNormalMatrix(S.modelViewMatrix),N.transparent===!0&&N.side===li&&N.forceSinglePass===!1?(N.side=tn,N.needsUpdate=!0,v.renderBufferDirect(O,U,F,N,S,ne),N.side=$i,N.needsUpdate=!0,v.renderBufferDirect(O,U,F,N,S,ne),N.side=li):v.renderBufferDirect(O,U,F,N,S,ne),S.onAfterRender(v,U,O,F,N,ne)}function _a(S,U,O){U.isScene!==!0&&(U=Ne);let F=be.get(S),N=d.state.lights,ne=d.state.shadowsArray,oe=N.state.version,pe=q.getParameters(S,N.state,ne,U,O),me=q.getProgramCacheKey(pe),Me=F.programs;F.environment=S.isMeshStandardMaterial?U.environment:null,F.fog=U.fog,F.envMap=(S.isMeshStandardMaterial?w:b).get(S.envMap||F.environment),F.envMapRotation=F.environment!==null&&S.envMap===null?U.environmentRotation:S.envMapRotation,Me===void 0&&(S.addEventListener("dispose",Se),Me=new Map,F.programs=Me);let Ee=Me.get(me);if(Ee!==void 0){if(F.currentProgram===Ee&&F.lightsStateVersion===oe)return Zp(S,pe),Ee}else pe.uniforms=q.getUniforms(S),S.onBeforeCompile(pe,v),Ee=q.acquireProgram(pe,me),Me.set(me,Ee),F.uniforms=pe.uniforms;let we=F.uniforms;return(!S.isShaderMaterial&&!S.isRawShaderMaterial||S.clipping===!0)&&(we.clippingPlanes=Ie.uniform),Zp(S,pe),F.needsLights=vy(S),F.lightsStateVersion=oe,F.needsLights&&(we.ambientLightColor.value=N.state.ambient,we.lightProbe.value=N.state.probe,we.directionalLights.value=N.state.directional,we.directionalLightShadows.value=N.state.directionalShadow,we.spotLights.value=N.state.spot,we.spotLightShadows.value=N.state.spotShadow,we.rectAreaLights.value=N.state.rectArea,we.ltc_1.value=N.state.rectAreaLTC1,we.ltc_2.value=N.state.rectAreaLTC2,we.pointLights.value=N.state.point,we.pointLightShadows.value=N.state.pointShadow,we.hemisphereLights.value=N.state.hemi,we.directionalShadowMap.value=N.state.directionalShadowMap,we.directionalShadowMatrix.value=N.state.directionalShadowMatrix,we.spotShadowMap.value=N.state.spotShadowMap,we.spotLightMatrix.value=N.state.spotLightMatrix,we.spotLightMap.value=N.state.spotLightMap,we.pointShadowMap.value=N.state.pointShadowMap,we.pointShadowMatrix.value=N.state.pointShadowMatrix),F.currentProgram=Ee,F.uniformsList=null,Ee}function $p(S){if(S.uniformsList===null){let U=S.currentProgram.getUniforms();S.uniformsList=Os.seqWithValue(U.seq,S.uniforms)}return S.uniformsList}function Zp(S,U){let O=be.get(S);O.outputColorSpace=U.outputColorSpace,O.batching=U.batching,O.batchingColor=U.batchingColor,O.instancing=U.instancing,O.instancingColor=U.instancingColor,O.instancingMorph=U.instancingMorph,O.skinning=U.skinning,O.morphTargets=U.morphTargets,O.morphNormals=U.morphNormals,O.morphColors=U.morphColors,O.morphTargetsCount=U.morphTargetsCount,O.numClippingPlanes=U.numClippingPlanes,O.numIntersection=U.numClipIntersection,O.vertexAlphas=U.vertexAlphas,O.vertexTangents=U.vertexTangents,O.toneMapping=U.toneMapping}function gy(S,U,O,F,N){U.isScene!==!0&&(U=Ne),Pe.resetTextureUnits();let ne=U.fog,oe=F.isMeshStandardMaterial?U.environment:null,pe=T===null?v.outputColorSpace:T.isXRRenderTarget===!0?T.texture.colorSpace:Ji,me=(F.isMeshStandardMaterial?w:b).get(F.envMap||oe),Me=F.vertexColors===!0&&!!O.attributes.color&&O.attributes.color.itemSize===4,Ee=!!O.attributes.tangent&&(!!F.normalMap||F.anisotropy>0),we=!!O.morphAttributes.position,Ye=!!O.morphAttributes.normal,ct=!!O.morphAttributes.color,ut=qi;F.toneMapped&&(T===null||T.isXRRenderTarget===!0)&&(ut=v.toneMapping);let rn=O.morphAttributes.position||O.morphAttributes.normal||O.morphAttributes.color,qe=rn!==void 0?rn.length:0,ve=be.get(F),At=d.state.lights;if(H===!0&&(Q===!0||S!==E)){let dn=S===E&&F.id===L;Ie.setState(F,S,dn)}let $e=!1;F.version===ve.__version?(ve.needsLights&&ve.lightsStateVersion!==At.state.version||ve.outputColorSpace!==pe||N.isBatchedMesh&&ve.batching===!1||!N.isBatchedMesh&&ve.batching===!0||N.isBatchedMesh&&ve.batchingColor===!0&&N.colorTexture===null||N.isBatchedMesh&&ve.batchingColor===!1&&N.colorTexture!==null||N.isInstancedMesh&&ve.instancing===!1||!N.isInstancedMesh&&ve.instancing===!0||N.isSkinnedMesh&&ve.skinning===!1||!N.isSkinnedMesh&&ve.skinning===!0||N.isInstancedMesh&&ve.instancingColor===!0&&N.instanceColor===null||N.isInstancedMesh&&ve.instancingColor===!1&&N.instanceColor!==null||N.isInstancedMesh&&ve.instancingMorph===!0&&N.morphTexture===null||N.isInstancedMesh&&ve.instancingMorph===!1&&N.morphTexture!==null||ve.envMap!==me||F.fog===!0&&ve.fog!==ne||ve.numClippingPlanes!==void 0&&(ve.numClippingPlanes!==Ie.numPlanes||ve.numIntersection!==Ie.numIntersection)||ve.vertexAlphas!==Me||ve.vertexTangents!==Ee||ve.morphTargets!==we||ve.morphNormals!==Ye||ve.morphColors!==ct||ve.toneMapping!==ut||ve.morphTargetsCount!==qe)&&($e=!0):($e=!0,ve.__version=F.version);let Cn=ve.currentProgram;$e===!0&&(Cn=_a(F,U,N));let Nr=!1,sn=!1,su=!1,_t=Cn.getUniforms(),gi=ve.uniforms;if(ye.useProgram(Cn.program)&&(Nr=!0,sn=!0,su=!0),F.id!==L&&(L=F.id,sn=!0),Nr||E!==S){_t.setValue(A,"projectionMatrix",S.projectionMatrix),_t.setValue(A,"viewMatrix",S.matrixWorldInverse);let dn=_t.map.cameraPosition;dn!==void 0&&dn.setValue(A,ce.setFromMatrixPosition(S.matrixWorld)),Qe.logarithmicDepthBuffer&&_t.setValue(A,"logDepthBufFC",2/(Math.log(S.far+1)/Math.LN2)),(F.isMeshPhongMaterial||F.isMeshToonMaterial||F.isMeshLambertMaterial||F.isMeshBasicMaterial||F.isMeshStandardMaterial||F.isShaderMaterial)&&_t.setValue(A,"isOrthographic",S.isOrthographicCamera===!0),E!==S&&(E=S,sn=!0,su=!0)}if(N.isSkinnedMesh){_t.setOptional(A,N,"bindMatrix"),_t.setOptional(A,N,"bindMatrixInverse");let dn=N.skeleton;dn&&(dn.boneTexture===null&&dn.computeBoneTexture(),_t.setValue(A,"boneTexture",dn.boneTexture,Pe))}N.isBatchedMesh&&(_t.setOptional(A,N,"batchingTexture"),_t.setValue(A,"batchingTexture",N._matricesTexture,Pe),_t.setOptional(A,N,"batchingIdTexture"),_t.setValue(A,"batchingIdTexture",N._indirectTexture,Pe),_t.setOptional(A,N,"batchingColorTexture"),N._colorsTexture!==null&&_t.setValue(A,"batchingColorTexture",N._colorsTexture,Pe));let ou=O.morphAttributes;if((ou.position!==void 0||ou.normal!==void 0||ou.color!==void 0)&&He.update(N,O,Cn),(sn||ve.receiveShadow!==N.receiveShadow)&&(ve.receiveShadow=N.receiveShadow,_t.setValue(A,"receiveShadow",N.receiveShadow)),F.isMeshGouraudMaterial&&F.envMap!==null&&(gi.envMap.value=me,gi.flipEnvMap.value=me.isCubeTexture&&me.isRenderTargetTexture===!1?-1:1),F.isMeshStandardMaterial&&F.envMap===null&&U.environment!==null&&(gi.envMapIntensity.value=U.environmentIntensity),sn&&(_t.setValue(A,"toneMappingExposure",v.toneMappingExposure),ve.needsLights&&_y(gi,su),ne&&F.fog===!0&&xe.refreshFogUniforms(gi,ne),xe.refreshMaterialUniforms(gi,F,K,V,d.state.transmissionRenderTarget[S.id]),Os.upload(A,$p(ve),gi,Pe)),F.isShaderMaterial&&F.uniformsNeedUpdate===!0&&(Os.upload(A,$p(ve),gi,Pe),F.uniformsNeedUpdate=!1),F.isSpriteMaterial&&_t.setValue(A,"center",N.center),_t.setValue(A,"modelViewMatrix",N.modelViewMatrix),_t.setValue(A,"normalMatrix",N.normalMatrix),_t.setValue(A,"modelMatrix",N.matrixWorld),F.isShaderMaterial||F.isRawShaderMaterial){let dn=F.uniformsGroups;for(let au=0,yy=dn.length;au<yy;au++){let Jp=dn[au];at.update(Jp,Cn),at.bind(Jp,Cn)}}return Cn}function _y(S,U){S.ambientLightColor.needsUpdate=U,S.lightProbe.needsUpdate=U,S.directionalLights.needsUpdate=U,S.directionalLightShadows.needsUpdate=U,S.pointLights.needsUpdate=U,S.pointLightShadows.needsUpdate=U,S.spotLights.needsUpdate=U,S.spotLightShadows.needsUpdate=U,S.rectAreaLights.needsUpdate=U,S.hemisphereLights.needsUpdate=U}function vy(S){return S.isMeshLambertMaterial||S.isMeshToonMaterial||S.isMeshPhongMaterial||S.isMeshStandardMaterial||S.isShadowMaterial||S.isShaderMaterial&&S.lights===!0}this.getActiveCubeFace=function(){return R},this.getActiveMipmapLevel=function(){return C},this.getRenderTarget=function(){return T},this.setRenderTargetTextures=function(S,U,O){be.get(S.texture).__webglTexture=U,be.get(S.depthTexture).__webglTexture=O;let F=be.get(S);F.__hasExternalTextures=!0,F.__autoAllocateDepthBuffer=O===void 0,F.__autoAllocateDepthBuffer||Je.has("WEBGL_multisampled_render_to_texture")===!0&&(console.warn("THREE.WebGLRenderer: Render-to-texture extension was disabled because an external texture was provided"),F.__useRenderToTexture=!1)},this.setRenderTargetFramebuffer=function(S,U){let O=be.get(S);O.__webglFramebuffer=U,O.__useDefaultFramebuffer=U===void 0},this.setRenderTarget=function(S,U=0,O=0){T=S,R=U,C=O;let F=!0,N=null,ne=!1,oe=!1;if(S){let me=be.get(S);me.__useDefaultFramebuffer!==void 0?(ye.bindFramebuffer(A.FRAMEBUFFER,null),F=!1):me.__webglFramebuffer===void 0?Pe.setupRenderTarget(S):me.__hasExternalTextures&&Pe.rebindTextures(S,be.get(S.texture).__webglTexture,be.get(S.depthTexture).__webglTexture);let Me=S.texture;(Me.isData3DTexture||Me.isDataArrayTexture||Me.isCompressedArrayTexture)&&(oe=!0);let Ee=be.get(S).__webglFramebuffer;S.isWebGLCubeRenderTarget?(Array.isArray(Ee[U])?N=Ee[U][O]:N=Ee[U],ne=!0):S.samples>0&&Pe.useMultisampledRTT(S)===!1?N=be.get(S).__webglMultisampledFramebuffer:Array.isArray(Ee)?N=Ee[O]:N=Ee,M.copy(S.viewport),P.copy(S.scissor),W=S.scissorTest}else M.copy(ge).multiplyScalar(K).floor(),P.copy(_e).multiplyScalar(K).floor(),W=Ve;if(ye.bindFramebuffer(A.FRAMEBUFFER,N)&&F&&ye.drawBuffers(S,N),ye.viewport(M),ye.scissor(P),ye.setScissorTest(W),ne){let me=be.get(S.texture);A.framebufferTexture2D(A.FRAMEBUFFER,A.COLOR_ATTACHMENT0,A.TEXTURE_CUBE_MAP_POSITIVE_X+U,me.__webglTexture,O)}else if(oe){let me=be.get(S.texture),Me=U||0;A.framebufferTextureLayer(A.FRAMEBUFFER,A.COLOR_ATTACHMENT0,me.__webglTexture,O||0,Me)}L=-1},this.readRenderTargetPixels=function(S,U,O,F,N,ne,oe){if(!(S&&S.isWebGLRenderTarget)){console.error("THREE.WebGLRenderer.readRenderTargetPixels: renderTarget is not THREE.WebGLRenderTarget.");return}let pe=be.get(S).__webglFramebuffer;if(S.isWebGLCubeRenderTarget&&oe!==void 0&&(pe=pe[oe]),pe){ye.bindFramebuffer(A.FRAMEBUFFER,pe);try{let me=S.texture,Me=me.format,Ee=me.type;if(!Qe.textureFormatReadable(Me)){console.error("THREE.WebGLRenderer.readRenderTargetPixels: renderTarget is not in RGBA or implementation defined format.");return}if(!Qe.textureTypeReadable(Ee)){console.error("THREE.WebGLRenderer.readRenderTargetPixels: renderTarget is not in UnsignedByteType or implementation defined type.");return}U>=0&&U<=S.width-F&&O>=0&&O<=S.height-N&&A.readPixels(U,O,F,N,Ae.convert(Me),Ae.convert(Ee),ne)}finally{let me=T!==null?be.get(T).__webglFramebuffer:null;ye.bindFramebuffer(A.FRAMEBUFFER,me)}}},this.readRenderTargetPixelsAsync=async function(S,U,O,F,N,ne,oe){if(!(S&&S.isWebGLRenderTarget))throw new Error("THREE.WebGLRenderer.readRenderTargetPixels: renderTarget is not THREE.WebGLRenderTarget.");let pe=be.get(S).__webglFramebuffer;if(S.isWebGLCubeRenderTarget&&oe!==void 0&&(pe=pe[oe]),pe){ye.bindFramebuffer(A.FRAMEBUFFER,pe);try{let me=S.texture,Me=me.format,Ee=me.type;if(!Qe.textureFormatReadable(Me))throw new Error("THREE.WebGLRenderer.readRenderTargetPixelsAsync: renderTarget is not in RGBA or implementation defined format.");if(!Qe.textureTypeReadable(Ee))throw new Error("THREE.WebGLRenderer.readRenderTargetPixelsAsync: renderTarget is not in UnsignedByteType or implementation defined type.");if(U>=0&&U<=S.width-F&&O>=0&&O<=S.height-N){let we=A.createBuffer();A.bindBuffer(A.PIXEL_PACK_BUFFER,we),A.bufferData(A.PIXEL_PACK_BUFFER,ne.byteLength,A.STREAM_READ),A.readPixels(U,O,F,N,Ae.convert(Me),Ae.convert(Ee),0),A.flush();let Ye=A.fenceSync(A.SYNC_GPU_COMMANDS_COMPLETE,0);await TS(A,Ye,4);try{A.bindBuffer(A.PIXEL_PACK_BUFFER,we),A.getBufferSubData(A.PIXEL_PACK_BUFFER,0,ne)}finally{A.deleteBuffer(we),A.deleteSync(Ye)}return ne}}finally{let me=T!==null?be.get(T).__webglFramebuffer:null;ye.bindFramebuffer(A.FRAMEBUFFER,me)}}},this.copyFramebufferToTexture=function(S,U=null,O=0){S.isTexture!==!0&&(console.warn("WebGLRenderer: copyFramebufferToTexture function signature has changed."),U=arguments[0]||null,S=arguments[1]);let F=Math.pow(2,-O),N=Math.floor(S.image.width*F),ne=Math.floor(S.image.height*F),oe=U!==null?U.x:0,pe=U!==null?U.y:0;Pe.setTexture2D(S,0),A.copyTexSubImage2D(A.TEXTURE_2D,O,0,0,oe,pe,N,ne),ye.unbindTexture()},this.copyTextureToTexture=function(S,U,O=null,F=null,N=0){S.isTexture!==!0&&(console.warn("WebGLRenderer: copyTextureToTexture function signature has changed."),F=arguments[0]||null,S=arguments[1],U=arguments[2],N=arguments[3]||0,O=null);let ne,oe,pe,me,Me,Ee;O!==null?(ne=O.max.x-O.min.x,oe=O.max.y-O.min.y,pe=O.min.x,me=O.min.y):(ne=S.image.width,oe=S.image.height,pe=0,me=0),F!==null?(Me=F.x,Ee=F.y):(Me=0,Ee=0);let we=Ae.convert(U.format),Ye=Ae.convert(U.type);Pe.setTexture2D(U,0),A.pixelStorei(A.UNPACK_FLIP_Y_WEBGL,U.flipY),A.pixelStorei(A.UNPACK_PREMULTIPLY_ALPHA_WEBGL,U.premultiplyAlpha),A.pixelStorei(A.UNPACK_ALIGNMENT,U.unpackAlignment);let ct=A.getParameter(A.UNPACK_ROW_LENGTH),ut=A.getParameter(A.UNPACK_IMAGE_HEIGHT),rn=A.getParameter(A.UNPACK_SKIP_PIXELS),qe=A.getParameter(A.UNPACK_SKIP_ROWS),ve=A.getParameter(A.UNPACK_SKIP_IMAGES),At=S.isCompressedTexture?S.mipmaps[N]:S.image;A.pixelStorei(A.UNPACK_ROW_LENGTH,At.width),A.pixelStorei(A.UNPACK_IMAGE_HEIGHT,At.height),A.pixelStorei(A.UNPACK_SKIP_PIXELS,pe),A.pixelStorei(A.UNPACK_SKIP_ROWS,me),S.isDataTexture?A.texSubImage2D(A.TEXTURE_2D,N,Me,Ee,ne,oe,we,Ye,At.data):S.isCompressedTexture?A.compressedTexSubImage2D(A.TEXTURE_2D,N,Me,Ee,At.width,At.height,we,At.data):A.texSubImage2D(A.TEXTURE_2D,N,Me,Ee,ne,oe,we,Ye,At),A.pixelStorei(A.UNPACK_ROW_LENGTH,ct),A.pixelStorei(A.UNPACK_IMAGE_HEIGHT,ut),A.pixelStorei(A.UNPACK_SKIP_PIXELS,rn),A.pixelStorei(A.UNPACK_SKIP_ROWS,qe),A.pixelStorei(A.UNPACK_SKIP_IMAGES,ve),N===0&&U.generateMipmaps&&A.generateMipmap(A.TEXTURE_2D),ye.unbindTexture()},this.copyTextureToTexture3D=function(S,U,O=null,F=null,N=0){S.isTexture!==!0&&(console.warn("WebGLRenderer: copyTextureToTexture3D function signature has changed."),O=arguments[0]||null,F=arguments[1]||null,S=arguments[2],U=arguments[3],N=arguments[4]||0);let ne,oe,pe,me,Me,Ee,we,Ye,ct,ut=S.isCompressedTexture?S.mipmaps[N]:S.image;O!==null?(ne=O.max.x-O.min.x,oe=O.max.y-O.min.y,pe=O.max.z-O.min.z,me=O.min.x,Me=O.min.y,Ee=O.min.z):(ne=ut.width,oe=ut.height,pe=ut.depth,me=0,Me=0,Ee=0),F!==null?(we=F.x,Ye=F.y,ct=F.z):(we=0,Ye=0,ct=0);let rn=Ae.convert(U.format),qe=Ae.convert(U.type),ve;if(U.isData3DTexture)Pe.setTexture3D(U,0),ve=A.TEXTURE_3D;else if(U.isDataArrayTexture||U.isCompressedArrayTexture)Pe.setTexture2DArray(U,0),ve=A.TEXTURE_2D_ARRAY;else{console.warn("THREE.WebGLRenderer.copyTextureToTexture3D: only supports THREE.DataTexture3D and THREE.DataTexture2DArray.");return}A.pixelStorei(A.UNPACK_FLIP_Y_WEBGL,U.flipY),A.pixelStorei(A.UNPACK_PREMULTIPLY_ALPHA_WEBGL,U.premultiplyAlpha),A.pixelStorei(A.UNPACK_ALIGNMENT,U.unpackAlignment);let At=A.getParameter(A.UNPACK_ROW_LENGTH),$e=A.getParameter(A.UNPACK_IMAGE_HEIGHT),Cn=A.getParameter(A.UNPACK_SKIP_PIXELS),Nr=A.getParameter(A.UNPACK_SKIP_ROWS),sn=A.getParameter(A.UNPACK_SKIP_IMAGES);A.pixelStorei(A.UNPACK_ROW_LENGTH,ut.width),A.pixelStorei(A.UNPACK_IMAGE_HEIGHT,ut.height),A.pixelStorei(A.UNPACK_SKIP_PIXELS,me),A.pixelStorei(A.UNPACK_SKIP_ROWS,Me),A.pixelStorei(A.UNPACK_SKIP_IMAGES,Ee),S.isDataTexture||S.isData3DTexture?A.texSubImage3D(ve,N,we,Ye,ct,ne,oe,pe,rn,qe,ut.data):U.isCompressedArrayTexture?A.compressedTexSubImage3D(ve,N,we,Ye,ct,ne,oe,pe,rn,ut.data):A.texSubImage3D(ve,N,we,Ye,ct,ne,oe,pe,rn,qe,ut),A.pixelStorei(A.UNPACK_ROW_LENGTH,At),A.pixelStorei(A.UNPACK_IMAGE_HEIGHT,$e),A.pixelStorei(A.UNPACK_SKIP_PIXELS,Cn),A.pixelStorei(A.UNPACK_SKIP_ROWS,Nr),A.pixelStorei(A.UNPACK_SKIP_IMAGES,sn),N===0&&U.generateMipmaps&&A.generateMipmap(ve),ye.unbindTexture()},this.initRenderTarget=function(S){be.get(S).__webglFramebuffer===void 0&&Pe.setupRenderTarget(S)},this.initTexture=function(S){S.isCubeTexture?Pe.setTextureCube(S,0):S.isData3DTexture?Pe.setTexture3D(S,0):S.isDataArrayTexture||S.isCompressedArrayTexture?Pe.setTexture2DArray(S,0):Pe.setTexture2D(S,0),ye.unbindTexture()},this.resetState=function(){R=0,C=0,T=null,ye.reset(),De.reset()},typeof __THREE_DEVTOOLS__<"u"&&__THREE_DEVTOOLS__.dispatchEvent(new CustomEvent("observe",{detail:this}))}get coordinateSystem(){return ui}get outputColorSpace(){return this._outputColorSpace}set outputColorSpace(e){this._outputColorSpace=e;let t=this.getContext();t.drawingBufferColorSpace=e===xp?"display-p3":"srgb",t.unpackColorSpace=Ke.workingColorSpace===Uc?"display-p3":"srgb"}};var Mc=class extends Vt{constructor(){super(),this.isScene=!0,this.type="Scene",this.background=null,this.environment=null,this.fog=null,this.backgroundBlurriness=0,this.backgroundIntensity=1,this.backgroundRotation=new Xn,this.environmentIntensity=1,this.environmentRotation=new Xn,this.overrideMaterial=null,typeof __THREE_DEVTOOLS__<"u"&&__THREE_DEVTOOLS__.dispatchEvent(new CustomEvent("observe",{detail:this}))}copy(e,t){return super.copy(e,t),e.background!==null&&(this.background=e.background.clone()),e.environment!==null&&(this.environment=e.environment.clone()),e.fog!==null&&(this.fog=e.fog.clone()),this.backgroundBlurriness=e.backgroundBlurriness,this.backgroundIntensity=e.backgroundIntensity,this.backgroundRotation.copy(e.backgroundRotation),this.environmentIntensity=e.environmentIntensity,this.environmentRotation.copy(e.environmentRotation),e.overrideMaterial!==null&&(this.overrideMaterial=e.overrideMaterial.clone()),this.matrixAutoUpdate=e.matrixAutoUpdate,this}toJSON(e){let t=super.toJSON(e);return this.fog!==null&&(t.object.fog=this.fog.toJSON()),this.backgroundBlurriness>0&&(t.object.backgroundBlurriness=this.backgroundBlurriness),this.backgroundIntensity!==1&&(t.object.backgroundIntensity=this.backgroundIntensity),t.object.backgroundRotation=this.backgroundRotation.toArray(),this.environmentIntensity!==1&&(t.object.environmentIntensity=this.environmentIntensity),t.object.environmentRotation=this.environmentRotation.toArray(),t}};var sa=class extends $t{constructor(e,t,i,r,s,o,a,l,c){super(e,t,i,r,s,o,a,l,c),this.isVideoTexture=!0,this.minFilter=o!==void 0?o:qt,this.magFilter=s!==void 0?s:qt,this.generateMipmaps=!1;let u=this;function p(){u.needsUpdate=!0,e.requestVideoFrameCallback(p)}"requestVideoFrameCallback"in e&&e.requestVideoFrameCallback(p)}clone(){return new this.constructor(this.image).copy(this)}update(){let e=this.image;"requestVideoFrameCallback"in e===!1&&e.readyState>=e.HAVE_CURRENT_DATA&&(this.needsUpdate=!0)}};var Ec=class extends $t{constructor(e,t,i,r,s,o,a,l,c){super(e,t,i,r,s,o,a,l,c),this.isCanvasTexture=!0,this.needsUpdate=!0}};var oa=class extends Ar{constructor(e){super(),this.isMeshStandardMaterial=!0,this.defines={STANDARD:""},this.type="MeshStandardMaterial",this.color=new Oe(16777215),this.roughness=1,this.metalness=0,this.map=null,this.lightMap=null,this.lightMapIntensity=1,this.aoMap=null,this.aoMapIntensity=1,this.emissive=new Oe(0),this.emissiveIntensity=1,this.emissiveMap=null,this.bumpMap=null,this.bumpScale=1,this.normalMap=null,this.normalMapType=Av,this.normalScale=new ze(1,1),this.displacementMap=null,this.displacementScale=1,this.displacementBias=0,this.roughnessMap=null,this.metalnessMap=null,this.alphaMap=null,this.envMap=null,this.envMapRotation=new Xn,this.envMapIntensity=1,this.wireframe=!1,this.wireframeLinewidth=1,this.wireframeLinecap="round",this.wireframeLinejoin="round",this.flatShading=!1,this.fog=!0,this.setValues(e)}copy(e){return super.copy(e),this.defines={STANDARD:""},this.color.copy(e.color),this.roughness=e.roughness,this.metalness=e.metalness,this.map=e.map,this.lightMap=e.lightMap,this.lightMapIntensity=e.lightMapIntensity,this.aoMap=e.aoMap,this.aoMapIntensity=e.aoMapIntensity,this.emissive.copy(e.emissive),this.emissiveMap=e.emissiveMap,this.emissiveIntensity=e.emissiveIntensity,this.bumpMap=e.bumpMap,this.bumpScale=e.bumpScale,this.normalMap=e.normalMap,this.normalMapType=e.normalMapType,this.normalScale.copy(e.normalScale),this.displacementMap=e.displacementMap,this.displacementScale=e.displacementScale,this.displacementBias=e.displacementBias,this.roughnessMap=e.roughnessMap,this.metalnessMap=e.metalnessMap,this.alphaMap=e.alphaMap,this.envMap=e.envMap,this.envMapRotation.copy(e.envMapRotation),this.envMapIntensity=e.envMapIntensity,this.wireframe=e.wireframe,this.wireframeLinewidth=e.wireframeLinewidth,this.wireframeLinecap=e.wireframeLinecap,this.wireframeLinejoin=e.wireframeLinejoin,this.flatShading=e.flatShading,this.fog=e.fog,this}};function ec(n,e,t){return!n||!t&&n.constructor===e?n:typeof e.BYTES_PER_ELEMENT=="number"?new e(n):Array.prototype.slice.call(n)}function yT(n){return ArrayBuffer.isView(n)&&!(n instanceof DataView)}var Hs=class{constructor(e,t,i,r){this.parameterPositions=e,this._cachedIndex=0,this.resultBuffer=r!==void 0?r:new t.constructor(i),this.sampleValues=t,this.valueSize=i,this.settings=null,this.DefaultSettings_={}}evaluate(e){let t=this.parameterPositions,i=this._cachedIndex,r=t[i],s=t[i-1];e:{t:{let o;n:{i:if(!(e<r)){for(let a=i+2;;){if(r===void 0){if(e<s)break i;return i=t.length,this._cachedIndex=i,this.copySampleValue_(i-1)}if(i===a)break;if(s=r,r=t[++i],e<r)break t}o=t.length;break n}if(!(e>=s)){let a=t[1];e<a&&(i=2,s=a);for(let l=i-2;;){if(s===void 0)return this._cachedIndex=0,this.copySampleValue_(0);if(i===l)break;if(r=s,s=t[--i-1],e>=s)break t}o=i,i=0;break n}break e}for(;i<o;){let a=i+o>>>1;e<t[a]?o=a:i=a+1}if(r=t[i],s=t[i-1],s===void 0)return this._cachedIndex=0,this.copySampleValue_(0);if(r===void 0)return i=t.length,this._cachedIndex=i,this.copySampleValue_(i-1)}this._cachedIndex=i,this.intervalChanged_(i,s,r)}return this.interpolate_(i,s,e,r)}getSettings_(){return this.settings||this.DefaultSettings_}copySampleValue_(e){let t=this.resultBuffer,i=this.sampleValues,r=this.valueSize,s=e*r;for(let o=0;o!==r;++o)t[o]=i[s+o];return t}interpolate_(){throw new Error("call to abstract method")}intervalChanged_(){}},Qf=class extends Hs{constructor(e,t,i,r){super(e,t,i,r),this._weightPrev=-0,this._offsetPrev=-0,this._weightNext=-0,this._offsetNext=-0,this.DefaultSettings_={endingStart:y_,endingEnd:y_}}intervalChanged_(e,t,i){let r=this.parameterPositions,s=e-2,o=e+1,a=r[s],l=r[o];if(a===void 0)switch(this.getSettings_().endingStart){case x_:s=e,a=2*t-i;break;case w_:s=r.length-2,a=t+r[s]-r[s+1];break;default:s=e,a=i}if(l===void 0)switch(this.getSettings_().endingEnd){case x_:o=e,l=2*i-t;break;case w_:o=1,l=i+r[1]-r[0];break;default:o=e-1,l=t}let c=(i-t)*.5,u=this.valueSize;this._weightPrev=c/(t-a),this._weightNext=c/(l-i),this._offsetPrev=s*u,this._offsetNext=o*u}interpolate_(e,t,i,r){let s=this.resultBuffer,o=this.sampleValues,a=this.valueSize,l=e*a,c=l-a,u=this._offsetPrev,p=this._offsetNext,f=this._weightPrev,m=this._weightNext,_=(i-t)/(r-t),y=_*_,d=y*_,h=-f*d+2*f*y-f*_,g=(1+f)*d+(-1.5-2*f)*y+(-.5+f)*_+1,v=(-1-m)*d+(1.5+m)*y+.5*_,x=m*d-m*y;for(let R=0;R!==a;++R)s[R]=h*o[u+R]+g*o[c+R]+v*o[l+R]+x*o[p+R];return s}},ep=class extends Hs{constructor(e,t,i,r){super(e,t,i,r)}interpolate_(e,t,i,r){let s=this.resultBuffer,o=this.sampleValues,a=this.valueSize,l=e*a,c=l-a,u=(i-t)/(r-t),p=1-u;for(let f=0;f!==a;++f)s[f]=o[c+f]*p+o[l+f]*u;return s}},tp=class extends Hs{constructor(e,t,i,r){super(e,t,i,r)}interpolate_(e){return this.copySampleValue_(e-1)}},Dn=class{constructor(e,t,i,r){if(e===void 0)throw new Error("THREE.KeyframeTrack: track name is undefined");if(t===void 0||t.length===0)throw new Error("THREE.KeyframeTrack: no keyframes in track named "+e);this.name=e,this.times=ec(t,this.TimeBufferType),this.values=ec(i,this.ValueBufferType),this.setInterpolation(r||this.DefaultInterpolation)}static toJSON(e){let t=e.constructor,i;if(t.toJSON!==this.toJSON)i=t.toJSON(e);else{i={name:e.name,times:ec(e.times,Array),values:ec(e.values,Array)};let r=e.getInterpolation();r!==e.DefaultInterpolation&&(i.interpolation=r)}return i.type=e.ValueTypeName,i}InterpolantFactoryMethodDiscrete(e){return new tp(this.times,this.values,this.getValueSize(),e)}InterpolantFactoryMethodLinear(e){return new ep(this.times,this.values,this.getValueSize(),e)}InterpolantFactoryMethodSmooth(e){return new Qf(this.times,this.values,this.getValueSize(),e)}setInterpolation(e){let t;switch(e){case ac:t=this.InterpolantFactoryMethodDiscrete;break;case Nf:t=this.InterpolantFactoryMethodLinear;break;case bd:t=this.InterpolantFactoryMethodSmooth;break}if(t===void 0){let i="unsupported interpolation for "+this.ValueTypeName+" keyframe track named "+this.name;if(this.createInterpolant===void 0)if(e!==this.DefaultInterpolation)this.setInterpolation(this.DefaultInterpolation);else throw new Error(i);return console.warn("THREE.KeyframeTrack:",i),this}return this.createInterpolant=t,this}getInterpolation(){switch(this.createInterpolant){case this.InterpolantFactoryMethodDiscrete:return ac;case this.InterpolantFactoryMethodLinear:return Nf;case this.InterpolantFactoryMethodSmooth:return bd}}getValueSize(){return this.values.length/this.times.length}shift(e){if(e!==0){let t=this.times;for(let i=0,r=t.length;i!==r;++i)t[i]+=e}return this}scale(e){if(e!==1){let t=this.times;for(let i=0,r=t.length;i!==r;++i)t[i]*=e}return this}trim(e,t){let i=this.times,r=i.length,s=0,o=r-1;for(;s!==r&&i[s]<e;)++s;for(;o!==-1&&i[o]>t;)--o;if(++o,s!==0||o!==r){s>=o&&(o=Math.max(o,1),s=o-1);let a=this.getValueSize();this.times=i.slice(s,o),this.values=this.values.slice(s*a,o*a)}return this}validate(){let e=!0,t=this.getValueSize();t-Math.floor(t)!==0&&(console.error("THREE.KeyframeTrack: Invalid value size in track.",this),e=!1);let i=this.times,r=this.values,s=i.length;s===0&&(console.error("THREE.KeyframeTrack: Track is empty.",this),e=!1);let o=null;for(let a=0;a!==s;a++){let l=i[a];if(typeof l=="number"&&isNaN(l)){console.error("THREE.KeyframeTrack: Time is not a valid number.",this,a,l),e=!1;break}if(o!==null&&o>l){console.error("THREE.KeyframeTrack: Out of order keys.",this,a,l,o),e=!1;break}o=l}if(r!==void 0&&yT(r))for(let a=0,l=r.length;a!==l;++a){let c=r[a];if(isNaN(c)){console.error("THREE.KeyframeTrack: Value is not a valid number.",this,a,c),e=!1;break}}return e}optimize(){let e=this.times.slice(),t=this.values.slice(),i=this.getValueSize(),r=this.getInterpolation()===bd,s=e.length-1,o=1;for(let a=1;a<s;++a){let l=!1,c=e[a],u=e[a+1];if(c!==u&&(a!==1||c!==e[0]))if(r)l=!0;else{let p=a*i,f=p-i,m=p+i;for(let _=0;_!==i;++_){let y=t[p+_];if(y!==t[f+_]||y!==t[m+_]){l=!0;break}}}if(l){if(a!==o){e[o]=e[a];let p=a*i,f=o*i;for(let m=0;m!==i;++m)t[f+m]=t[p+m]}++o}}if(s>0){e[o]=e[s];for(let a=s*i,l=o*i,c=0;c!==i;++c)t[l+c]=t[a+c];++o}return o!==e.length?(this.times=e.slice(0,o),this.values=t.slice(0,o*i)):(this.times=e,this.values=t),this}clone(){let e=this.times.slice(),t=this.values.slice(),i=this.constructor,r=new i(this.name,e,t);return r.createInterpolant=this.createInterpolant,r}};Dn.prototype.TimeBufferType=Float32Array;Dn.prototype.ValueBufferType=Float32Array;Dn.prototype.DefaultInterpolation=Nf;var Ir=class extends Dn{constructor(e,t,i){super(e,t,i)}};Ir.prototype.ValueTypeName="bool";Ir.prototype.ValueBufferType=Array;Ir.prototype.DefaultInterpolation=ac;Ir.prototype.InterpolantFactoryMethodLinear=void 0;Ir.prototype.InterpolantFactoryMethodSmooth=void 0;var np=class extends Dn{};np.prototype.ValueTypeName="color";var ip=class extends Dn{};ip.prototype.ValueTypeName="number";var rp=class extends Hs{constructor(e,t,i,r){super(e,t,i,r)}interpolate_(e,t,i,r){let s=this.resultBuffer,o=this.sampleValues,a=this.valueSize,l=(i-t)/(r-t),c=e*a;for(let u=c+a;c!==u;c+=4)Nn.slerpFlat(s,0,o,c-a,o,c,l);return s}},Cc=class extends Dn{InterpolantFactoryMethodLinear(e){return new rp(this.times,this.values,this.getValueSize(),e)}};Cc.prototype.ValueTypeName="quaternion";Cc.prototype.InterpolantFactoryMethodSmooth=void 0;var Lr=class extends Dn{constructor(e,t,i){super(e,t,i)}};Lr.prototype.ValueTypeName="string";Lr.prototype.ValueBufferType=Array;Lr.prototype.DefaultInterpolation=ac;Lr.prototype.InterpolantFactoryMethodLinear=void 0;Lr.prototype.InterpolantFactoryMethodSmooth=void 0;var sp=class extends Dn{};sp.prototype.ValueTypeName="vector";var Tc={enabled:!1,files:{},add:function(n,e){this.enabled!==!1&&(this.files[n]=e)},get:function(n){if(this.enabled!==!1)return this.files[n]},remove:function(n){delete this.files[n]},clear:function(){this.files={}}},op=class{constructor(e,t,i){let r=this,s=!1,o=0,a=0,l,c=[];this.onStart=void 0,this.onLoad=e,this.onProgress=t,this.onError=i,this.itemStart=function(u){a++,s===!1&&r.onStart!==void 0&&r.onStart(u,o,a),s=!0},this.itemEnd=function(u){o++,r.onProgress!==void 0&&r.onProgress(u,o,a),o===a&&(s=!1,r.onLoad!==void 0&&r.onLoad())},this.itemError=function(u){r.onError!==void 0&&r.onError(u)},this.resolveURL=function(u){return l?l(u):u},this.setURLModifier=function(u){return l=u,this},this.addHandler=function(u,p){return c.push(u,p),this},this.removeHandler=function(u){let p=c.indexOf(u);return p!==-1&&c.splice(p,2),this},this.getHandler=function(u){for(let p=0,f=c.length;p<f;p+=2){let m=c[p],_=c[p+1];if(m.global&&(m.lastIndex=0),m.test(u))return _}return null}}},xT=new op,Gs=class{constructor(e){this.manager=e!==void 0?e:xT,this.crossOrigin="anonymous",this.withCredentials=!1,this.path="",this.resourcePath="",this.requestHeader={}}load(){}loadAsync(e,t){let i=this;return new Promise(function(r,s){i.load(e,r,t,s)})}parse(){}setCrossOrigin(e){return this.crossOrigin=e,this}setWithCredentials(e){return this.withCredentials=e,this}setPath(e){return this.path=e,this}setResourcePath(e){return this.resourcePath=e,this}setRequestHeader(e){return this.requestHeader=e,this}};Gs.DEFAULT_MATERIAL_NAME="__DEFAULT";var oi={},ap=class extends Error{constructor(e,t){super(e),this.response=t}},bc=class extends Gs{constructor(e){super(e)}load(e,t,i,r){e===void 0&&(e=""),this.path!==void 0&&(e=this.path+e),e=this.manager.resolveURL(e);let s=Tc.get(e);if(s!==void 0)return this.manager.itemStart(e),setTimeout(()=>{t&&t(s),this.manager.itemEnd(e)},0),s;if(oi[e]!==void 0){oi[e].push({onLoad:t,onProgress:i,onError:r});return}oi[e]=[],oi[e].push({onLoad:t,onProgress:i,onError:r});let o=new Request(e,{headers:new Headers(this.requestHeader),credentials:this.withCredentials?"include":"same-origin"}),a=this.mimeType,l=this.responseType;fetch(o).then(c=>{if(c.status===200||c.status===0){if(c.status===0&&console.warn("THREE.FileLoader: HTTP Status 0 received."),typeof ReadableStream>"u"||c.body===void 0||c.body.getReader===void 0)return c;let u=oi[e],p=c.body.getReader(),f=c.headers.get("X-File-Size")||c.headers.get("Content-Length"),m=f?parseInt(f):0,_=m!==0,y=0,d=new ReadableStream({start(h){g();function g(){p.read().then(({done:v,value:x})=>{if(v)h.close();else{y+=x.byteLength;let R=new ProgressEvent("progress",{lengthComputable:_,loaded:y,total:m});for(let C=0,T=u.length;C<T;C++){let L=u[C];L.onProgress&&L.onProgress(R)}h.enqueue(x),g()}},v=>{h.error(v)})}}});return new Response(d)}else throw new ap(`fetch for "${c.url}" responded with ${c.status}: ${c.statusText}`,c)}).then(c=>{switch(l){case"arraybuffer":return c.arrayBuffer();case"blob":return c.blob();case"document":return c.text().then(u=>new DOMParser().parseFromString(u,a));case"json":return c.json();default:if(a===void 0)return c.text();{let p=/charset="?([^;"\s]*)"?/i.exec(a),f=p&&p[1]?p[1].toLowerCase():void 0,m=new TextDecoder(f);return c.arrayBuffer().then(_=>m.decode(_))}}}).then(c=>{Tc.add(e,c);let u=oi[e];delete oi[e];for(let p=0,f=u.length;p<f;p++){let m=u[p];m.onLoad&&m.onLoad(c)}}).catch(c=>{let u=oi[e];if(u===void 0)throw this.manager.itemError(e),c;delete oi[e];for(let p=0,f=u.length;p<f;p++){let m=u[p];m.onError&&m.onError(c)}this.manager.itemError(e)}).finally(()=>{this.manager.itemEnd(e)}),this.manager.itemStart(e)}setResponseType(e){return this.responseType=e,this}setMimeType(e){return this.mimeType=e,this}};var lp=class extends Gs{constructor(e){super(e)}load(e,t,i,r){this.path!==void 0&&(e=this.path+e),e=this.manager.resolveURL(e);let s=this,o=Tc.get(e);if(o!==void 0)return s.manager.itemStart(e),setTimeout(function(){t&&t(o),s.manager.itemEnd(e)},0),o;let a=na("img");function l(){u(),Tc.add(e,this),t&&t(this),s.manager.itemEnd(e)}function c(p){u(),r&&r(p),s.manager.itemError(e),s.manager.itemEnd(e)}function u(){a.removeEventListener("load",l,!1),a.removeEventListener("error",c,!1)}return a.addEventListener("load",l,!1),a.addEventListener("error",c,!1),e.slice(0,5)!=="data:"&&this.crossOrigin!==void 0&&(a.crossOrigin=this.crossOrigin),s.manager.itemStart(e),a.src=e,a}};var Ac=class extends Gs{constructor(e){super(e)}load(e,t,i,r){let s=new $t,o=new lp(this.manager);return o.setCrossOrigin(this.crossOrigin),o.setPath(this.path),o.load(e,function(a){s.image=a,s.needsUpdate=!0,t!==void 0&&t(s)},i,r),s}},Rc=class extends Vt{constructor(e,t=1){super(),this.isLight=!0,this.type="Light",this.color=new Oe(e),this.intensity=t}dispose(){}copy(e,t){return super.copy(e,t),this.color.copy(e.color),this.intensity=e.intensity,this}toJSON(e){let t=super.toJSON(e);return t.object.color=this.color.getHex(),t.object.intensity=this.intensity,this.groundColor!==void 0&&(t.object.groundColor=this.groundColor.getHex()),this.distance!==void 0&&(t.object.distance=this.distance),this.angle!==void 0&&(t.object.angle=this.angle),this.decay!==void 0&&(t.object.decay=this.decay),this.penumbra!==void 0&&(t.object.penumbra=this.penumbra),this.shadow!==void 0&&(t.object.shadow=this.shadow.toJSON()),this.target!==void 0&&(t.object.target=this.target.uuid),t}};var ef=new et,fv=new D,pv=new D,cp=class{constructor(e){this.camera=e,this.intensity=1,this.bias=0,this.normalBias=0,this.radius=1,this.blurSamples=8,this.mapSize=new ze(512,512),this.map=null,this.mapPass=null,this.matrix=new et,this.autoUpdate=!0,this.needsUpdate=!1,this._frustum=new ra,this._frameExtents=new ze(1,1),this._viewportCount=1,this._viewports=[new wt(0,0,1,1)]}getViewportCount(){return this._viewportCount}getFrustum(){return this._frustum}updateMatrices(e){let t=this.camera,i=this.matrix;fv.setFromMatrixPosition(e.matrixWorld),t.position.copy(fv),pv.setFromMatrixPosition(e.target.matrixWorld),t.lookAt(pv),t.updateMatrixWorld(),ef.multiplyMatrices(t.projectionMatrix,t.matrixWorldInverse),this._frustum.setFromProjectionMatrix(ef),i.set(.5,0,0,.5,0,.5,0,.5,0,0,.5,.5,0,0,0,1),i.multiply(ef)}getViewport(e){return this._viewports[e]}getFrameExtents(){return this._frameExtents}dispose(){this.map&&this.map.dispose(),this.mapPass&&this.mapPass.dispose()}copy(e){return this.camera=e.camera.clone(),this.intensity=e.intensity,this.bias=e.bias,this.radius=e.radius,this.mapSize.copy(e.mapSize),this}clone(){return new this.constructor().copy(this)}toJSON(){let e={};return this.intensity!==1&&(e.intensity=this.intensity),this.bias!==0&&(e.bias=this.bias),this.normalBias!==0&&(e.normalBias=this.normalBias),this.radius!==1&&(e.radius=this.radius),(this.mapSize.x!==512||this.mapSize.y!==512)&&(e.mapSize=this.mapSize.toArray()),e.camera=this.camera.toJSON(!1).object,delete e.camera.matrix,e}};var up=class extends cp{constructor(){super(new yc(-5,5,5,-5,.5,500)),this.isDirectionalLightShadow=!0}},Pc=class extends Rc{constructor(e,t){super(e,t),this.isDirectionalLight=!0,this.type="DirectionalLight",this.position.copy(Vt.DEFAULT_UP),this.updateMatrix(),this.target=new Vt,this.shadow=new up}dispose(){this.shadow.dispose()}copy(e){return super.copy(e),this.target=e.target.clone(),this.shadow=e.shadow.clone(),this}},Ic=class extends Rc{constructor(e,t){super(e,t),this.isAmbientLight=!0,this.type="AmbientLight"}};var Sp="\\[\\]\\.:\\/",wT=new RegExp("["+Sp+"]","g"),Mp="[^"+Sp+"]",ST="[^"+Sp.replace("\\.","")+"]",MT=/((?:WC+[\/:])*)/.source.replace("WC",Mp),ET=/(WCOD+)?/.source.replace("WCOD",ST),CT=/(?:\.(WC+)(?:\[(.+)\])?)?/.source.replace("WC",Mp),TT=/\.(WC+)(?:\[(.+)\])?/.source.replace("WC",Mp),bT=new RegExp("^"+MT+ET+CT+TT+"$"),AT=["material","materials","bones","map"],hp=class{constructor(e,t,i){let r=i||ot.parseTrackName(t);this._targetGroup=e,this._bindings=e.subscribe_(t,r)}getValue(e,t){this.bind();let i=this._targetGroup.nCachedObjects_,r=this._bindings[i];r!==void 0&&r.getValue(e,t)}setValue(e,t){let i=this._bindings;for(let r=this._targetGroup.nCachedObjects_,s=i.length;r!==s;++r)i[r].setValue(e,t)}bind(){let e=this._bindings;for(let t=this._targetGroup.nCachedObjects_,i=e.length;t!==i;++t)e[t].bind()}unbind(){let e=this._bindings;for(let t=this._targetGroup.nCachedObjects_,i=e.length;t!==i;++t)e[t].unbind()}},ot=class n{constructor(e,t,i){this.path=t,this.parsedPath=i||n.parseTrackName(t),this.node=n.findNode(e,this.parsedPath.nodeName),this.rootNode=e,this.getValue=this._getValue_unbound,this.setValue=this._setValue_unbound}static create(e,t,i){return e&&e.isAnimationObjectGroup?new n.Composite(e,t,i):new n(e,t,i)}static sanitizeNodeName(e){return e.replace(/\s/g,"_").replace(wT,"")}static parseTrackName(e){let t=bT.exec(e);if(t===null)throw new Error("PropertyBinding: Cannot parse trackName: "+e);let i={nodeName:t[2],objectName:t[3],objectIndex:t[4],propertyName:t[5],propertyIndex:t[6]},r=i.nodeName&&i.nodeName.lastIndexOf(".");if(r!==void 0&&r!==-1){let s=i.nodeName.substring(r+1);AT.indexOf(s)!==-1&&(i.nodeName=i.nodeName.substring(0,r),i.objectName=s)}if(i.propertyName===null||i.propertyName.length===0)throw new Error("PropertyBinding: can not parse propertyName from trackName: "+e);return i}static findNode(e,t){if(t===void 0||t===""||t==="."||t===-1||t===e.name||t===e.uuid)return e;if(e.skeleton){let i=e.skeleton.getBoneByName(t);if(i!==void 0)return i}if(e.children){let i=function(s){for(let o=0;o<s.length;o++){let a=s[o];if(a.name===t||a.uuid===t)return a;let l=i(a.children);if(l)return l}return null},r=i(e.children);if(r)return r}return null}_getValue_unavailable(){}_setValue_unavailable(){}_getValue_direct(e,t){e[t]=this.targetObject[this.propertyName]}_getValue_array(e,t){let i=this.resolvedProperty;for(let r=0,s=i.length;r!==s;++r)e[t++]=i[r]}_getValue_arrayElement(e,t){e[t]=this.resolvedProperty[this.propertyIndex]}_getValue_toArray(e,t){this.resolvedProperty.toArray(e,t)}_setValue_direct(e,t){this.targetObject[this.propertyName]=e[t]}_setValue_direct_setNeedsUpdate(e,t){this.targetObject[this.propertyName]=e[t],this.targetObject.needsUpdate=!0}_setValue_direct_setMatrixWorldNeedsUpdate(e,t){this.targetObject[this.propertyName]=e[t],this.targetObject.matrixWorldNeedsUpdate=!0}_setValue_array(e,t){let i=this.resolvedProperty;for(let r=0,s=i.length;r!==s;++r)i[r]=e[t++]}_setValue_array_setNeedsUpdate(e,t){let i=this.resolvedProperty;for(let r=0,s=i.length;r!==s;++r)i[r]=e[t++];this.targetObject.needsUpdate=!0}_setValue_array_setMatrixWorldNeedsUpdate(e,t){let i=this.resolvedProperty;for(let r=0,s=i.length;r!==s;++r)i[r]=e[t++];this.targetObject.matrixWorldNeedsUpdate=!0}_setValue_arrayElement(e,t){this.resolvedProperty[this.propertyIndex]=e[t]}_setValue_arrayElement_setNeedsUpdate(e,t){this.resolvedProperty[this.propertyIndex]=e[t],this.targetObject.needsUpdate=!0}_setValue_arrayElement_setMatrixWorldNeedsUpdate(e,t){this.resolvedProperty[this.propertyIndex]=e[t],this.targetObject.matrixWorldNeedsUpdate=!0}_setValue_fromArray(e,t){this.resolvedProperty.fromArray(e,t)}_setValue_fromArray_setNeedsUpdate(e,t){this.resolvedProperty.fromArray(e,t),this.targetObject.needsUpdate=!0}_setValue_fromArray_setMatrixWorldNeedsUpdate(e,t){this.resolvedProperty.fromArray(e,t),this.targetObject.matrixWorldNeedsUpdate=!0}_getValue_unbound(e,t){this.bind(),this.getValue(e,t)}_setValue_unbound(e,t){this.bind(),this.setValue(e,t)}bind(){let e=this.node,t=this.parsedPath,i=t.objectName,r=t.propertyName,s=t.propertyIndex;if(e||(e=n.findNode(this.rootNode,t.nodeName),this.node=e),this.getValue=this._getValue_unavailable,this.setValue=this._setValue_unavailable,!e){console.warn("THREE.PropertyBinding: No target node found for track: "+this.path+".");return}if(i){let c=t.objectIndex;switch(i){case"materials":if(!e.material){console.error("THREE.PropertyBinding: Can not bind to material as node does not have a material.",this);return}if(!e.material.materials){console.error("THREE.PropertyBinding: Can not bind to material.materials as node.material does not have a materials array.",this);return}e=e.material.materials;break;case"bones":if(!e.skeleton){console.error("THREE.PropertyBinding: Can not bind to bones as node does not have a skeleton.",this);return}e=e.skeleton.bones;for(let u=0;u<e.length;u++)if(e[u].name===c){c=u;break}break;case"map":if("map"in e){e=e.map;break}if(!e.material){console.error("THREE.PropertyBinding: Can not bind to material as node does not have a material.",this);return}if(!e.material.map){console.error("THREE.PropertyBinding: Can not bind to material.map as node.material does not have a map.",this);return}e=e.material.map;break;default:if(e[i]===void 0){console.error("THREE.PropertyBinding: Can not bind to objectName of node undefined.",this);return}e=e[i]}if(c!==void 0){if(e[c]===void 0){console.error("THREE.PropertyBinding: Trying to bind to objectIndex of objectName, but is undefined.",this,e);return}e=e[c]}}let o=e[r];if(o===void 0){let c=t.nodeName;console.error("THREE.PropertyBinding: Trying to update property for track: "+c+"."+r+" but it wasn't found.",e);return}let a=this.Versioning.None;this.targetObject=e,e.needsUpdate!==void 0?a=this.Versioning.NeedsUpdate:e.matrixWorldNeedsUpdate!==void 0&&(a=this.Versioning.MatrixWorldNeedsUpdate);let l=this.BindingType.Direct;if(s!==void 0){if(r==="morphTargetInfluences"){if(!e.geometry){console.error("THREE.PropertyBinding: Can not bind to morphTargetInfluences because node does not have a geometry.",this);return}if(!e.geometry.morphAttributes){console.error("THREE.PropertyBinding: Can not bind to morphTargetInfluences because node does not have a geometry.morphAttributes.",this);return}e.morphTargetDictionary[s]!==void 0&&(s=e.morphTargetDictionary[s])}l=this.BindingType.ArrayElement,this.resolvedProperty=o,this.propertyIndex=s}else o.fromArray!==void 0&&o.toArray!==void 0?(l=this.BindingType.HasFromToArray,this.resolvedProperty=o):Array.isArray(o)?(l=this.BindingType.EntireArray,this.resolvedProperty=o):this.propertyName=r;this.getValue=this.GetterByBindingType[l],this.setValue=this.SetterByBindingTypeAndVersioning[l][a]}unbind(){this.node=null,this.getValue=this._getValue_unbound,this.setValue=this._setValue_unbound}};ot.Composite=hp;ot.prototype.BindingType={Direct:0,EntireArray:1,ArrayElement:2,HasFromToArray:3};ot.prototype.Versioning={None:0,NeedsUpdate:1,MatrixWorldNeedsUpdate:2};ot.prototype.GetterByBindingType=[ot.prototype._getValue_direct,ot.prototype._getValue_array,ot.prototype._getValue_arrayElement,ot.prototype._getValue_toArray];ot.prototype.SetterByBindingTypeAndVersioning=[[ot.prototype._setValue_direct,ot.prototype._setValue_direct_setNeedsUpdate,ot.prototype._setValue_direct_setMatrixWorldNeedsUpdate],[ot.prototype._setValue_array,ot.prototype._setValue_array_setNeedsUpdate,ot.prototype._setValue_array_setMatrixWorldNeedsUpdate],[ot.prototype._setValue_arrayElement,ot.prototype._setValue_arrayElement_setNeedsUpdate,ot.prototype._setValue_arrayElement_setMatrixWorldNeedsUpdate],[ot.prototype._setValue_fromArray,ot.prototype._setValue_fromArray_setNeedsUpdate,ot.prototype._setValue_fromArray_setMatrixWorldNeedsUpdate]];var Sb=new Float32Array(1);typeof __THREE_DEVTOOLS__<"u"&&__THREE_DEVTOOLS__.dispatchEvent(new CustomEvent("register",{detail:{revision:dp}}));typeof window<"u"&&(window.__THREE__?console.warn("WARNING: Multiple instances of Three.js being imported."):window.__THREE__=dp);var Fe=class n{static formatDate(e){let t=e.getFullYear(),i=("0"+(e.getMonth()+1)).slice(-2),r=("0"+e.getDate()).slice(-2),s=("0"+e.getHours()).slice(-2),o=("0"+e.getMinutes()).slice(-2),a=("0"+e.getSeconds()).slice(-2);return`${t}-${i}-${r} ${s}:${o}:${a}.${e.getMilliseconds()}`}static serverLog(e){try{n.post("/log",{log:e})}catch{}}static getNowSecond(){return new Date().getTime()/1e3}static postRes(e,t){return new Promise(i=>{fetch(e,{body:JSON.stringify(t),headers:{"Content-Type":"application/json"},method:"POST"}).then(r=>{r.json().then(s=>{i(s)})})})}static post(e,t){return fetch(e,{body:JSON.stringify(t),headers:{"Content-Type":"application/json"},method:"POST"})}static formatLog(e){return`${this.formatDate(new Date)} ${e}`}static formatServerLog(e,t){return`[client-${e}-${this.formatDate(new Date)}]${t}`}static info(e,t=void 0){console.log(this.formatLog(e)),this.serverLog(this.formatServerLog("info",e)),t!==void 0&&console.log(t)}static warn(e,t=void 0){console.warn(this.formatLog(e)),this.serverLog(this.formatServerLog("warn",e)),t!==void 0&&console.warn(t)}static error(e,t=void 0){console.error(this.formatLog(e)),this.serverLog(this.formatServerLog("error",e)),t!==void 0&&console.error(t)}static createMatrixFromArray(e){return new et(e[0],e[4],e[8],e[12],e[1],e[5],e[9],e[13],e[2],e[6],e[10],e[14],e[3],e[7],e[11],e[15])}static formatMatrix(e){let t=new et;return t.compose(new D(e.position.x,e.position.y,e.position.z),new Nn(e.orientation.x,e.orientation.y,e.orientation.z,e.orientation.w),new D(1,1,1)),t}static decomposeMatrix(e){let t=new D,i=new Nn,r=new D;return e.decompose(t,i,r),{position:t,quaternion:i,scale:r}}static getXVector(e){return new D(e.elements[0],e.elements[1],e.elements[2])}static getYVector(e){return new D(e.elements[4],e.elements[5],e.elements[6])}static getZVector(e){return new D(e.elements[8],e.elements[9],e.elements[10])}static getWorldPos(e){return new D(e.elements[12],e.elements[13],e.elements[14])}static getLocalStorage(e){return localStorage.getItem(e)}static setLocalStorage(e,t){localStorage.setItem(e,t)}};var kv=class n{static UNKNOWN="UNKNOWN";static CAMERA_STEREO="CAMERA_STEREO";static CAMERA_MONOCULAR="CAMERA_MONOCULAR";static SAPIEN_STEREO="SAPIEN_STEREO";static SAPIEN_MONOCULAR="SAPIEN_MONOCULAR";static MUJUCO_MONOCULAR="MUJUCO_MONOCULAR";static SHARE_MEMORY_STEREO="SHARE_MEMORY_STEREO";static LIST_ALL=[n.CAMERA_STEREO,n.CAMERA_MONOCULAR,n.SAPIEN_STEREO,n.MUJUCO_MONOCULAR,n.SHARE_MEMORY_STEREO]},ua=class{m_pc;m_video;m_strSource;m_nWidth;m_nHeight;m_nLastFrameTime=1;m_listTime=[];m_divText;m_funOnVideoFrame=void 0;m_funGetApp;m_dictConfig={};constructor(){}init(e,t,i,r,s,o=void 0){this.m_strSource=e,this.m_nWidth=t,this.m_nHeight=i,this.m_dictConfig=r,this.m_video=s,this.m_video.requestVideoFrameCallback(this.loopVideoFrame.bind(this)),this.m_divText=document.getElementById("txt_frame"),this.m_funOnVideoFrame=o}getVideo(){return this.m_video}loopVideoFrame(e,t){let i=Fe.getNowSecond(),r=i-this.m_nLastFrameTime;this.m_listTime.push(r),this.m_nLastFrameTime=i,this.m_listTime.length>100&&this.m_listTime.splice(0,1);let s=0;for(let o of this.m_listTime)s+=o;s!==0&&(this.m_divText.innerText=`FPS: ${Math.floor(1/s*this.m_listTime.length)}`),this.onVideoFrame(e,t),this.m_video.requestVideoFrameCallback(this.loopVideoFrame.bind(this))}onVideoFrame(e,t){this.m_funOnVideoFrame&&this.m_funOnVideoFrame()}async negotiate(){return this.m_pc.addTransceiver("video",{direction:"recvonly"}),this.m_pc.onicecandidate=e=>{console.warn("onicecandidate",e)},this.m_pc.createOffer().then(e=>this.m_pc.setLocalDescription(e)).then(()=>new Promise(e=>{if(this.m_pc.iceGatheringState==="complete")e();else{let t=()=>{console.warn("ice",this.m_pc.iceGatheringState),this.m_pc.iceGatheringState==="complete"&&(this.m_pc.removeEventListener("icegatheringstatechange",t),e())};this.m_pc.addEventListener("icegatheringstatechange",t)}})).then(()=>{let e=this.m_pc.localDescription;return Fe.post("/offer",{sdp:e.sdp,type:e.type,source:this.m_strSource,width:this.m_nWidth,height:this.m_nHeight,config:this.m_dictConfig})}).then(e=>e.json()).then(e=>(this.m_funGetApp&&this.m_funGetApp().showVRButton(!0),console.log(e),this.m_pc.setRemoteDescription(e))).catch(e=>{alert(e)})}setFunGetApp(e){this.m_funGetApp=e}async startRTC(){let e={iceServers:[{urls:"stun:stun.127.0.0.1"}]};this.m_pc=new RTCPeerConnection(e),this.m_pc.addEventListener("track",t=>{if(t.track.kind==="video"){let i=t.streams[0];this.m_video.srcObject=i,Fe.info(`[${this.m_strSource}] RTCPeerConnection video track: ${this.m_video.srcObject}`)}}),Fe.serverLog("track"),await this.negotiate()}stop(){setTimeout(()=>{this.m_pc.close()},500)}};var RT=["left","right"],zv=[0,1,4,5],Dc=class{_list_pose_ip=[];_str_pose_ip="None";_dict_last_pose={left:[0,0,0,1,0,0,0],right:[0,0,0,1,0,0,0]};m_dictVRInput={headPose:new et,headLinearVelocity:{x:0,y:0,z:0,w:0},headAngularVelocity:{x:0,y:0,z:0,w:0},rightPose:new et,rightAxes:[],rightButton:[],rightLinearVelocity:{x:0,y:0,z:0,w:0},rightAngularVelocity:{x:0,y:0,z:0,w:0},leftAxes:[],leftButton:[],leftPose:new et,leftLinearVelocity:{x:0,y:0,z:0,w:0},leftAngularVelocity:{x:0,y:0,z:0,w:0}};m_dictButtonState={left:{},right:{}};m_dictButtonUp={left:{},right:{}};m_dictButtonDown={left:{},right:{}};m_dictButtonVibrate={left:{duration:0,intensity:0},right:{duration:0,intensity:0}};_str_state="NO_DATA";_battery_mgr=void 0;_n_data_time=new Date().getTime();constructor(){for(let e of zv)this.m_dictButtonState.left[e.toString()]=0,this.m_dictButtonState.right[e.toString()]=0}set_vr_state(e){this._str_state=e}update_data_time(){this._n_data_time=new Date().getTime()}get_vr_state(){return this._str_state}get_pose_ip(){return this._str_pose_ip}set_list_pose_ip(e){this._list_pose_ip=e}change_list_pose_ip(e){if(this._list_pose_ip.length<=0)return;let t=this._list_pose_ip.indexOf(this._str_pose_ip);if(t<0){this._str_pose_ip=this._list_pose_ip[0];return}e>0?(t=(t+1)%this._list_pose_ip.length,this._str_pose_ip=this._list_pose_ip[t],Fe.serverLog(`\u4E0B\u4E00\u4E2Aip: ${this._str_pose_ip}`)):e<0&&(t-=1,t<0&&(t=this._list_pose_ip.length+1),this._str_pose_ip=this._list_pose_ip[t],Fe.serverLog(`\u4E0A\u4E00\u4E2Aip: ${this._str_pose_ip}`))}get_list_pose_ip(){return this._list_pose_ip}set_pose_ip(e){this._str_pose_ip=e}getAxesValue(e,t){return e==="Left"?this.m_dictVRInput.leftAxes[t]:this.m_dictVRInput.rightAxes[t]}onXRFrame(e,t){try{let i=!0,r=e.getViewerPose(t);this.m_dictVRInput.headPose=Fe.formatMatrix(r.transform),r.linearVelocity&&(this.m_dictVRInput.headLinearVelocity=r.linearVelocity),r.angularVelocity&&(this.m_dictVRInput.headAngularVelocity=r.angularVelocity);let s=e.session.inputSources;if(s.length===1){this.set_vr_state("SINGLE"),this.update_data_time();return}else if(s.length<=0){this.set_vr_state("NO_DATA");return}else this.set_vr_state("NORMAL"),this.update_data_time();let o=15;for(let a of s){if(a.hand){this.set_vr_state("HAND_TEST"),this.update_data_time();continue}let l=e.getPose(a.gripSpace,t),c=[],u=[];if(a.gamepad&&(u=a.gamepad.axes.map(p=>p),c=a.gamepad.buttons.map(p=>({value:p.value,pressed:p.pressed,touched:p.touched}))),!l){i=!1;continue}if(!l.transform){i=!1;continue}if(a.handedness==="left"){let p=[l.transform.orientation.x,l.transform.orientation.y,l.transform.orientation.z,l.transform.orientation.w,l.transform.position.x,l.transform.position.y,l.transform.position.z],f=this._dict_last_pose.left;this._dict_last_pose.left=p,this.m_dictVRInput.leftPose=Fe.formatMatrix(l.transform),this.m_dictVRInput.leftButton=c,this.m_dictVRInput.leftAxes=u,l.linearVelocity&&(this.m_dictVRInput.leftLinearVelocity=l.linearVelocity),l.angularVelocity&&(this.m_dictVRInput.leftAngularVelocity=l.angularVelocity)}else if(a.handedness==="right"){let p=[l.transform.orientation.x,l.transform.orientation.y,l.transform.orientation.z,l.transform.orientation.w,l.transform.position.x,l.transform.position.y,l.transform.position.z],f=this._dict_last_pose.right;this._dict_last_pose.right=p,this.m_dictVRInput.rightPose=Fe.formatMatrix(l.transform),this.m_dictVRInput.rightButton=c,this.m_dictVRInput.rightAxes=u,l.linearVelocity&&(this.m_dictVRInput.rightLinearVelocity=l.linearVelocity),l.angularVelocity&&(this.m_dictVRInput.rightAngularVelocity=l.angularVelocity)}}for(let a of s){let l=this.m_dictButtonVibrate[a.handedness].intensity,c=this.m_dictButtonVibrate[a.handedness].duration;if(l<=0||c<=0)continue;let u=a.gamepad;u.hapticActuators?(u.hapticActuators[0].pulse(l,c),Fe.info(`\u624B\u67C4\u9707\u52A8:${a.handedness},${l}, ${c}`)):Fe.info("\u624B\u67C4\u4E0D\u5177\u6709\u9707\u52A8\u529F\u80FD:"+a.handedness)}this.m_dictButtonVibrate.left={duration:0,intensity:0},this.m_dictButtonVibrate.right={duration:0,intensity:0}}catch(i){i&&Fe.error(`onXRFrame error ${i.toString()}`)}}async sendVRData(){new Date().getTime()-this._n_data_time<2e3||(this._str_state="NO_DATA"),this._battery_mgr||(this._battery_mgr=await navigator.getBattery());try{let t=Fe.decomposeMatrix(this.m_dictVRInput.headPose),i=Fe.decomposeMatrix(this.m_dictVRInput.leftPose),r=Fe.decomposeMatrix(this.m_dictVRInput.rightPose),s={state:this.get_vr_state(),battery:this._battery_mgr.level,head:{position:{x:t.position.x,y:t.position.y,z:t.position.z},rotation:{x:t.quaternion.x,y:t.quaternion.y,z:t.quaternion.z,w:t.quaternion.w},linearVelocity:this.m_dictVRInput.headLinearVelocity,angularVelocity:this.m_dictVRInput.headAngularVelocity},left:{position:{x:i.position.x,y:i.position.y,z:i.position.z},rotation:{x:i.quaternion.x,y:i.quaternion.y,z:i.quaternion.z,w:i.quaternion.w},button:this.m_dictVRInput.leftButton,axes:this.m_dictVRInput.leftAxes,linearVelocity:this.m_dictVRInput.leftLinearVelocity,angularVelocity:this.m_dictVRInput.leftAngularVelocity},right:{position:{x:r.position.x,y:r.position.y,z:r.position.z},rotation:{x:r.quaternion.x,y:r.quaternion.y,z:r.quaternion.z,w:r.quaternion.w},button:this.m_dictVRInput.rightButton,axes:this.m_dictVRInput.rightAxes,linearVelocity:this.m_dictVRInput.rightLinearVelocity,angularVelocity:this.m_dictVRInput.rightAngularVelocity}};this._str_pose_ip!==void 0&&this._str_pose_ip!=="None"&&Fe.post(`https://${this._str_pose_ip}:5000/poseData`,s)}catch{}}setVibrate(e,t,i){this.m_dictButtonVibrate[e].duration=i*1e3,this.m_dictButtonVibrate[e].intensity=t}async requestMsg(){let t=(await Fe.postRes("/msg",{})).msg;for(let i of t)if(i.id==="vibrate"){let r=i.data,s=r.side,o=r.intensity,a=r.duration;this.setVibrate(s,o,a)}}updateFrame(e){this.requestMsg(),this.checkButtonChange()}registerBtnUp(e,t,i){let r=t.toString();r in this.m_dictButtonUp[e]||(this.m_dictButtonUp[e][r]=[]),this.m_dictButtonUp[e][r].push(i)}registerBtnDown(e,t,i){let r=t.toString();r in this.m_dictButtonDown[e]||(this.m_dictButtonDown[e][r]=[]),this.m_dictButtonDown[e][r].push(i)}checkButtonChange(){for(let e of RT){let t=this.m_dictButtonState[e],i=this.m_dictButtonUp[e],r=this.m_dictButtonDown[e];for(let s of zv){let o=s.toString(),a=t[o],l=[];e==="left"?l=this.m_dictVRInput.leftButton:e==="right"&&(l=this.m_dictVRInput.rightButton);let c=l[s];if(!c)continue;let u=c.value;if(u!==a)if(this.m_dictButtonState[e][o]=u,u>.5){let p=r[o];if(p)for(let f of p)f()}else{let p=i[o];if(p)for(let f of p)f()}}}}};var IT=.02,LT=.04,UT=.04,NT=4,Vv=.002,Hv=.1,Oc=class{m_videoLeft;m_videoRight;m_textureLeft;m_textureRight;m_planeLeft;m_planeRight;m_nXDistance=.028;m_nZDistance=.05;init(e,t,i=void 0){this.m_videoLeft=e,this.m_videoRight=t;let r=120,o=.06/2*2,a=o*.75;this.m_nXDistance=.028,this.m_nZDistance=-1;let l=new Yn(o,a);this.m_textureLeft=new sa(e),this.m_textureLeft.colorSpace=un;let c=new Rr({map:this.m_textureLeft});this.m_planeLeft=new St(l,c),this.m_planeLeft.layers.set(1),this.m_textureRight=new sa(t),this.m_textureRight.colorSpace=un;let u=new Rr({map:this.m_textureRight});this.m_planeRight=new St(l,u),this.m_planeRight.layers.set(2),this.updateDistance()}updateDistance(){this.m_planeLeft.position.set(-this.m_nXDistance,0,-this.m_nZDistance),this.m_planeRight.position.set(this.m_nXDistance,0,-this.m_nZDistance),Fe.setLocalStorage("eyeX",this.m_nXDistance.toString()),Fe.setLocalStorage("eyeZ",this.m_nZDistance.toString())}setToScene(e,t){t?(e.add(this.m_planeLeft),e.add(this.m_planeRight)):(e.remove(this.m_planeLeft),e.remove(this.m_planeRight))}play(){this.m_videoLeft&&this.m_videoLeft.play(),this.m_videoRight&&this.m_videoRight.play()}updateFrame(e,t){let i=t.getComVRInput(),r=i.getAxesValue("Left",2),s=i.getAxesValue("Left",3),o=!1;Math.abs(r)>.8&&(r<0&&this.m_nXDistance<=LT?this.m_nXDistance+=Vv*e:r>0&&this.m_nXDistance>=IT&&(this.m_nXDistance-=Vv*e),o=!0),Math.abs(s)>.8&&(s<0&&this.m_nZDistance<=NT?this.m_nZDistance+=Hv*e:s>0&&this.m_nZDistance>=UT&&(this.m_nZDistance-=Hv*e),o=!0),o&&this.updateDistance()}};var Ut={};Ut.d=(n,e)=>{for(var t in e)Ut.o(e,t)&&!Ut.o(n,t)&&Object.defineProperty(n,t,{enumerable:!0,get:e[t]})};Ut.o=(n,e)=>Object.prototype.hasOwnProperty.call(n,e);Ut.r=n=>{typeof Symbol<"u"&&Symbol.toStringTag&&Object.defineProperty(n,Symbol.toStringTag,{value:"Module"}),Object.defineProperty(n,"__esModule",{value:!0})};var hn={};Ut.d(hn,{g1:()=>Xc,gO:()=>Ur,km:()=>Wc,zV:()=>da,ol:()=>pa,uM:()=>Yc,N1:()=>Gc,xv:()=>fa,PH:()=>$c,UH:()=>qc,ZP:()=>lb,Vx:()=>fy});var Wc={};Ut.r(Wc);Ut.d(Wc,{COLUMN:()=>Zc,COLUMN_REVERSE:()=>qv,ROW:()=>ha,ROW_REVERSE:()=>Yv,contentDirection:()=>$v});var Xc={};Ut.r(Xc);Ut.d(Xc,{CENTER:()=>Pp,END:()=>Ip,START:()=>Rp,STRETCH:()=>Zv,alignItems:()=>Jv,warnAboutDeprecatedAlignItems:()=>Kv});var Yc={};Ut.r(Yc);Ut.d(Yc,{CENTER:()=>Up,END:()=>Np,SPACE_AROUND:()=>Dp,SPACE_BETWEEN:()=>Op,SPACE_EVENLY:()=>Fp,START:()=>Lp,justifyContent:()=>jv});var qc={};Ut.r(qc);Ut.d(qc,{NORMAL:()=>ma,NOWRAP:()=>Ys,PRE:()=>Jc,PRE_LINE:()=>qs,PRE_WRAP:()=>Kc,WHITE_CHARS:()=>Cp,collapseWhitespaceOnInlines:()=>ny,collapseWhitespaceOnString:()=>Qv,newlineBreakability:()=>ey,shouldBreak:()=>ty});var $c={};Ut.r($c);Ut.d($c,{CENTER:()=>kp,JUSTIFY:()=>zp,JUSTIFY_CENTER:()=>ay,JUSTIFY_LEFT:()=>oy,JUSTIFY_RIGHT:()=>Vc,LEFT:()=>ry,RIGHT:()=>sy,textAlign:()=>ly});var DT=n=>{var e={};return Ut.d(e,n),e};var ke=DT({BufferAttribute:()=>nn,BufferGeometry:()=>pi,CanvasTexture:()=>Ec,Color:()=>Oe,FileLoader:()=>bc,LinearFilter:()=>qt,Mesh:()=>St,Object3D:()=>Vt,Plane:()=>Ln,PlaneGeometry:()=>Yn,ShaderMaterial:()=>Mn,TextureLoader:()=>Ac,Vector2:()=>ze,Vector3:()=>D}),ha="row",Yv="row-reverse",Zc="column",qv="column-reverse";function $v(n,e,t,i){let r=t,s="getWidth",o="x",a="y";e.indexOf(Zc)===0&&(s="getHeight",o="y",a="x");for(let l=0;l<n.childrenBoxes.length;l++){let c=n.childrenBoxes[l],u=c.id,p=c[s](),f=c.margin||0;r+=f*i,n.childrenPos[u]={[o]:r+p/2*i,[a]:0},r+=i*(p+f)}}var Rp="start",Pp="center",Ip="end",Zv="stretch";function Jv(n,e){let t=n.getAlignItems();OT.indexOf(t)===-1&&console.warn(`alignItems === '${t}' is not supported`);let i="getWidth",r="x";e.indexOf(ha)===0&&(i="getHeight",r="y");let s=n[i]()/2-(n.padding||0);n.childrenBoxes.forEach(o=>{let a;switch(t){case Ip:case"right":case"bottom":e.indexOf(ha)===0?a=-s+o[i]()/2+(o.margin||0):a=s-o[i]()/2-(o.margin||0);break;case Rp:case"left":case"top":e.indexOf(ha)===0?a=s-o[i]()/2-(o.margin||0):a=-s+o[i]()/2+(o.margin||0);break}n.childrenPos[o.id][r]=a||0})}function Kv(n){FT.indexOf(n)!==-1&&console.warn(`alignItems === '${n}' is deprecated and will be remove in 7.x.x. Fallback are 'start'|'end'`)}var OT=[Rp,Pp,Ip,Zv,"top","right","bottom","left"],FT=["top","right","bottom","left"],Lp="start",Up="center",Np="end",Dp="space-around",Op="space-between",Fp="space-evenly";function jv(n,e,t,i){let r=n.getJustifyContent();BT.indexOf(r)===-1&&console.warn(`justifyContent === '${r}' is not supported`);let s=e.indexOf("row")===0?"width":"height",o=n.getChildrenSideSum(s),l=(s==="width"?n.getInnerWidth():n.getInnerHeight())-o,c=t*2-o*Math.sign(t),u=kT(r,c),p=zT(n.childrenBoxes,l,r,i),f=e.indexOf("row")===0?"x":"y";n.childrenBoxes.forEach((m,_)=>{n.childrenPos[m.id][f]-=u-p[_]})}var BT=[Lp,Up,Np,Dp,Op,Fp];function kT(n,e){switch(n){case Np:return e;case Up:return e/2}return 0}function zT(n,e,t,i){let r=Array(n.length).fill(0);if(e>0)switch(t){case Op:if(n.length>1){let s=e/(n.length-1)*i;r[0]=0;for(let o=1;o<n.length;o++)r[o]=s*o}break;case Fp:if(n.length>1){let s=e/(n.length+1)*i;for(let o=0;o<n.length;o++)r[o]=s*(o+1)}break;case Dp:if(n.length>1){let s=e/n.length*i,o=s/2;r[0]=o;for(let a=1;a<n.length;a++)r[a]=o+s*a}break}return r}function Bp(n){return class extends n{constructor(t){super(t),this.isBoxComponent=!0,this.childrenPos={}}getInnerWidth(){let t=this.getContentDirection();switch(t){case"row":case"row-reverse":return this.width-(this.padding*2||0)||this.getChildrenSideSum("width");case"column":case"column-reverse":return this.getHighestChildSizeOn("width");default:console.error(`Invalid contentDirection : ${t}`);break}}getInnerHeight(){let t=this.getContentDirection();switch(t){case"row":case"row-reverse":return this.getHighestChildSizeOn("height");case"column":case"column-reverse":return this.height-(this.padding*2||0)||this.getChildrenSideSum("height");default:console.error(`Invalid contentDirection : ${t}`);break}}getChildrenSideSum(t){return this.childrenBoxes.reduce((i,r)=>{let s=r.margin*2||0,o=t==="width"?r.getWidth()+s:r.getHeight()+s;return i+o},0)}setPosFromParentRecords(){this.parentUI&&this.parentUI.childrenPos[this.id]&&(this.position.x=this.parentUI.childrenPos[this.id].x,this.position.y=this.parentUI.childrenPos[this.id].y)}computeChildrenPosition(){if(this.children.length>0){let t=this.getContentDirection(),i;switch(t){case ha:i=-this.getInnerWidth()/2;break;case Yv:i=this.getInnerWidth()/2;break;case Zc:i=this.getInnerHeight()/2;break;case qv:i=-this.getInnerHeight()/2;break}let r=-Math.sign(i);$v(this,t,i,r),jv(this,t,i,r),Jv(this,t)}}getHighestChildSizeOn(t){return this.childrenBoxes.reduce((i,r)=>{let s=r.margin||0,o=t==="width"?r.getWidth()+s*2:r.getHeight()+s*2;return Math.max(i,o)},0)}getWidth(){return this.parentUI&&this.parentUI.getAlignItems()==="stretch"&&this.parentUI.getContentDirection().indexOf("column")!==-1?this.parentUI.getWidth()-(this.parentUI.padding*2||0):this.width||this.getInnerWidth()+(this.padding*2||0)}getHeight(){return this.parentUI&&this.parentUI.getAlignItems()==="stretch"&&this.parentUI.getContentDirection().indexOf("row")!==-1?this.parentUI.getHeight()-(this.parentUI.padding*2||0):this.height||this.getInnerHeight()+(this.padding*2||0)}}}var Cp={"	":"	","\n":`
`,"\r":"\r"," ":" "},ma="normal",Ys="nowrap",Jc="pre",qs="pre-line",Kc="pre-wrap",Qv=function(n,e){switch(e){case Ys:case ma:n=n.replace(/\n/g," ");case qs:n=n.replace(/[ ]{2,}/g," ");break;default:}return n},ey=function(n){switch(n){case Jc:case Kc:case qs:return"mandatory";case Ys:case ma:default:}},ty=function(n,e,t,i){let r=n[e];switch(i.WHITESPACE){case ma:case qs:case Kc:if(r.lineBreak==="mandatory")return!0;let s=r.kerning?r.kerning:0,o=r.xoffset?r.xoffset:0,a=r.xadvance?r.xadvance:r.width;if(t+a+o+s>i.INNER_WIDTH)return!0;let l=iy(n,e,i);return VT(n[e-1],t,l,i);case Jc:return r.lineBreak==="mandatory";case Ys:default:return!1}},ny=function(n,e){let t=n[0],i=n[n.length-1];switch(e){case Kc:t.glyph&&t.glyph===`
`&&n.length>1&&Wv([t],n[1]),i.glyph&&i.glyph===`
`&&n.length>1&&Gv([i],n[n.length-2]);break;case qs:case Ys:case ma:let r=[],s;for(let o=0;o<n.length;o++){let a=n[o];if(a.glyph&&Cp[a.glyph]&&n.length>o){r.push(a),s=n[o+1];continue}break}Wv(r,s),r=[],s=null;for(let o=n.length-1;o>0;o--){let a=n[o];if(a.glyph&&Cp[a.glyph]&&o>0){r.push(a),s=n[o-1];continue}break}Gv(r,s);break;case Jc:break;default:return console.warn(`whiteSpace: '${e}' is not valid`),0}return t.offsetX};function Gv(n,e){if(e)for(let t=0;t<n.length;t++){let i=n[t];i.width=0,i.height=0,i.offsetX=e.offsetX+e.width}}function Wv(n,e){if(e)for(let t=0;t<n.length;t++){let i=n[t];i.width=0,i.height=0,i.offsetX=e.offsetX}}function iy(n,e,t,i){if(i=i||0,!n[e])return i;let r=n[e],s=r.kerning?r.kerning:0,o=r.xoffset?r.xoffset:0,a=r.xadvance?r.xadvance:r.width;return r.lineBreak?i+a:iy(n,e+1,t,i+a+t.LETTERSPACING+o+s)}function VT(n,e,t,i){return!n||!n.glyph||e+t<i.INNER_WIDTH?!1:i.BREAKON.indexOf(n.glyph)>-1}var ry="left",sy="right",kp="center",zp="justify",oy="justify-left",Vc="justify-right",ay="justify-center";function ly(n,e,t){for(let i=0;i<n.length;i++){let r=n[i],s=HT(r,e,t,i===n.length-1);for(let o=0;o<r.length;o++)r[o].offsetX+=s;r.x=s}if(e.indexOf(zp)===0)for(let i=0;i<n.length;i++){let r=n[i];if(e.indexOf("-")!==-1&&i===n.length-1)return;let s=t-r.width;if(s<=0)return;let o=0;for(let u=1;u<r.length-1;u++)o+=r[u].glyph===" "?1:0;let a=s/o,l=1;e===Vc&&(r.reverse(),l=-1);let c=0;for(let u=1;u<=r.length-1;u++){let p=r[u];p.offsetX+=c*l,c+=p.glyph===" "?a:0}e===Vc&&r.reverse()}}var HT=(n,e,t,i)=>{switch(e){case oy:case zp:case ry:return-t/2;case Vc:case sy:return-n.width+t/2;case kp:return-n.width/2;case ay:return i?-n.width/2:-t/2;default:console.warn(`textAlign: '${e}' is not valid`)}};function cy(n){return class extends n{computeInlinesPosition(){let t=this.getWidth()-(this.padding*2||0),i=this.getHeight()-(this.padding*2||0),r=this.getJustifyContent(),s=this.getTextAlign(),o=this.getInterLine(),a=this.computeLines();a.interLine=o;let l=Math.abs(a.height),c=(()=>{switch(r){case"start":return i/2;case"end":return l-i/2;case"center":return l/2;default:console.warn(`justifyContent: '${r}' is not valid`)}})();a.forEach(u=>{u.y+=c,u.forEach(p=>{p.offsetY+=c})}),ly(a,s,t),this.lines=a}calculateBestFit(t){if(this.childrenInlines.length!==0)switch(t){case"grow":this.calculateGrowFit();break;case"shrink":this.calculateShrinkFit();break;case"auto":this.calculateAutoFit();break}}calculateGrowFit(){let t=this.getHeight()-(this.padding*2||0),i=1,r=.075,s=this.childrenInlines.find(u=>u.isText),o=1,a=2,l=s._fitFontSize?s._fitFontSize/s.getFontSize():1,c;do if(c=this.calculateHeight(l),c>t){if(l<=o){this.childrenInlines.forEach(u=>{u.isInlineBlock||(u._fitFontSize=u.getFontSize())});break}a=l,l-=(a-o)/2}else{if(Math.abs(t-c)<r)break;Math.abs(l-a)<5e-10&&(a*=2),o=l,l+=(a-o)/2}while(++i<=10)}calculateShrinkFit(){let t=this.getHeight()-(this.padding*2||0),i=1,r=.075,s=this.childrenInlines.find(u=>u.isText),o=0,a=1,l=s._fitFontSize?s._fitFontSize/s.getFontSize():1,c;do if(c=this.calculateHeight(l),c>t)a=l,l-=(a-o)/2;else{if(l>=a){this.childrenInlines.forEach(u=>{u.isInlineBlock||(u._fitFontSize=u.getFontSize())});break}if(Math.abs(t-c)<r)break;o=l,l+=(a-o)/2}while(++i<=10)}calculateAutoFit(){let t=this.getHeight()-(this.padding*2||0),i=1,r=.075,s=this.childrenInlines.find(u=>u.isText),o=0,a=2,l=s._fitFontSize?s._fitFontSize/s.getFontSize():1,c;do if(c=this.calculateHeight(l),c>t)a=l,l-=(a-o)/2;else{if(Math.abs(t-c)<r)break;Math.abs(l-a)<5e-10&&(a*=2),o=l,l+=(a-o)/2}while(++i<=10)}computeLines(){let t=this.getWidth()-(this.padding*2||0),i=[[]];i.height=0;let r=this.getInterLine();this.childrenInlines.reduce((l,c)=>{if(!c.inlines)return;let u=c._fitFontSize||c.getFontSize(),p=c.isText?c.getLetterSpacing()*u:0,f=c.getWhiteSpace(),m=c.getBreakOn(),_={WHITESPACE:f,LETTERSPACING:p,BREAKON:m,INNER_WIDTH:t};return c.inlines.reduce((d,h,g,v)=>{let x=h.kerning?h.kerning:0,R=h.xoffset?h.xoffset:0,C=h.xadvance?h.xadvance:h.width;return ty(v,g,d,_)?(i.push([h]),h.offsetX=R,h.width===0?0:C+p):(i[i.length-1].push(h),h.offsetX=d+R+x,d+C+x+p)},l)},0);let s=0,o=0,a=-r/2;return i.forEach(l=>{if(l.lineHeight=l.reduce((u,p)=>{let f=p.lineHeight!==void 0?p.lineHeight:p.height;return Math.max(u,f)},0),l.lineBase=l.reduce((u,p)=>{let f=p.lineBase!==void 0?p.lineBase:p.height;return Math.max(u,f)},0),l.width=0,l.height=l.lineHeight,l[0]){let u=this.getWhiteSpace(),p=ny(l,u);l.forEach(f=>{f.offsetX-=p}),l.width=this.computeLineWidth(l),l.width>s&&(s=l.width),l.forEach(f=>{f.offsetY=a-f.height-f.anchor,f.lineHeight<l.lineHeight&&(f.offsetY-=l.lineBase-f.lineBase)}),l.y=a,o+=l.lineHeight+r,a=a-(l.lineHeight+r)}}),i.height=o,i.width=s,i}calculateHeight(t){this.childrenInlines.forEach(r=>{r.isInlineBlock||(r._fitFontSize=r.getFontSize()*t,r.calculateInlines(r._fitFontSize))});let i=this.computeLines();return Math.abs(i.height)}computeLineWidth(t){let i=t[0],r=t[t.length-1];return r.offsetX+r.width+i.offsetX}}}var GT=new ke.FileLoader,Tp=[],Bc={},WT=new ke.TextureLoader,bp=[],kc={},Ht={};function XT(n,e){typeof e=="string"?qT(n,e):(Ht[n.id]||(Ht[n.id]={component:n}),Vp(e),Ht[n.id].json=e,n._updateFontFamily(e))}function YT(n,e){bp.indexOf(e)===-1&&(bp.push(e),WT.load(e,t=>{t.generateMipmaps=!1,t.minFilter=ke.LinearFilter,t.magFilter=ke.LinearFilter,kc[e]=t;for(let i of Object.keys(Ht))e===Ht[i].textureURL&&Ht[i].component._updateFontTexture(t)})),Ht[n.id]||(Ht[n.id]={component:n}),Ht[n.id].textureURL=e,kc[e]&&n._updateFontTexture(kc[e])}function uy(n){let e=Ht[n.id];return!e&&n.parentUI?uy(n.parentUI):e}function qT(n,e){Tp.indexOf(e)===-1&&(Tp.push(e),GT.load(e,t=>{let i=JSON.parse(t);Vp(i),Bc[e]=i;for(let r of Object.keys(Ht))e===Ht[r].jsonURL&&Ht[r].component._updateFontFamily(i)})),Ht[n.id]||(Ht[n.id]={component:n}),Ht[n.id].jsonURL=e,Bc[e]&&n._updateFontFamily(Bc[e])}function Vp(n){if(n._kernings)return;let e={};for(let t=0;t<n.kernings.length;t++){let i=n.kernings[t];if(i.amount===0)continue;let r=String.fromCharCode(i.first,i.second);e[r]=i.amount}n._kernings=e}function $T(n,e,t){t.generateMipmaps=!1,t.minFilter=ke.LinearFilter,t.magFilter=ke.LinearFilter,Tp.push(n),Bc[n]=e,Vp(e),t&&(bp.push(n),kc[n]=t)}var ZT={setFontFamily:XT,setFontTexture:YT,getFontOf:uy,addFont:$T},da=ZT,mi=class{static requestUpdate(e,t,i,r){e.traverse(s=>{s.isUI&&(this.requestedUpdates[s.id]?(t&&(this.requestedUpdates[s.id].updateParsing=!0),i&&(this.requestedUpdates[s.id].updateLayout=!0),r&&(this.requestedUpdates[s.id].updateInner=!0)):this.requestedUpdates[s.id]={updateParsing:t,updateLayout:i,updateInner:r,needCallback:t||i||r})})}static register(e){this.components.includes(e)||this.components.push(e)}static disposeOf(e){let t=this.components.indexOf(e);t>-1&&this.components.splice(t,1)}static update(){if(Object.keys(this.requestedUpdates).length>0){let e=this.components.filter(t=>!t.parentUI);e.forEach(t=>this.traverseParsing(t)),e.forEach(t=>this.traverseUpdates(t))}}static traverseParsing(e){let t=this.requestedUpdates[e.id];t&&t.updateParsing&&(e.parseParams(),t.updateParsing=!1),e.childrenUIs.forEach(i=>this.traverseParsing(i))}static traverseUpdates(e){let t=this.requestedUpdates[e.id];delete this.requestedUpdates[e.id],t&&t.updateLayout&&(t.updateLayout=!1,e.updateLayout()),t&&t.updateInner&&(t.updateInner=!1,e.updateInner()),e.childrenUIs.forEach(i=>{this.traverseUpdates(i)}),t&&t.needCallback&&e.onAfterUpdate()}};mi.components=[];mi.requestedUpdates={};var Lt={container:null,fontFamily:null,fontSize:.05,fontKerning:"normal",bestFit:"none",offset:.01,interLine:.01,breakOn:`- ,.:?!
`,whiteSpace:qs,contentDirection:Zc,alignItems:Pp,justifyContent:Lp,textAlign:kp,textType:"MSDF",fontColor:new ke.Color(16777215),fontOpacity:1,fontPXRange:4,fontSupersampling:!0,borderRadius:.01,borderWidth:0,borderColor:new ke.Color("black"),borderOpacity:1,backgroundSize:"cover",backgroundColor:new ke.Color(2236962),backgroundWhiteColor:new ke.Color(16777215),backgroundOpacity:.8,backgroundOpaqueOpacity:1,getDefaultTexture:JT,hiddenOverflow:!1,letterSpacing:0},Fc;function JT(){if(!Fc){let n=document.createElement("canvas").getContext("2d");n.canvas.width=1,n.canvas.height=1,n.fillStyle="#ffffff",n.fillRect(0,0,1,1),Fc=new ke.CanvasTexture(n.canvas),Fc.isDefault=!0}return Fc}function jc(n){return class extends n{constructor(t){super(t),this.states={},this.currentState=void 0,this.isUI=!0,this.autoLayout=!0,this.childrenUIs=[],this.childrenBoxes=[],this.childrenTexts=[],this.childrenInlines=[],this.parentUI=null,this.addEventListener("added",this._rebuildParentUI),this.addEventListener("removed",this._rebuildParentUI)}getClippingPlanes(){let t=[];if(this.parentUI){if(this.isBlock&&this.parentUI.getHiddenOverflow()){let i=this.parentUI.getHeight()/2-(this.parentUI.padding||0),r=this.parentUI.getWidth()/2-(this.parentUI.padding||0),s=[new ke.Plane(new ke.Vector3(0,1,0),i),new ke.Plane(new ke.Vector3(0,-1,0),i),new ke.Plane(new ke.Vector3(1,0,0),r),new ke.Plane(new ke.Vector3(-1,0,0),r)];s.forEach(o=>{o.applyMatrix4(this.parent.matrixWorld)}),t.push(...s)}this.parentUI.parentUI&&t.push(...this.parentUI.getClippingPlanes())}return t}getHighestParent(){return this.parentUI?this.parent.getHighestParent():this}_getProperty(t){return this[t]===void 0&&this.parentUI?this.parent._getProperty(t):this[t]!==void 0?this[t]:Lt[t]}getFontSize(){return this._getProperty("fontSize")}getFontKerning(){return this._getProperty("fontKerning")}getLetterSpacing(){return this._getProperty("letterSpacing")}getFontTexture(){return this.fontTexture===void 0&&this.parentUI?this.parent._getProperty("fontTexture"):this.fontTexture!==void 0?this.fontTexture:Lt.getDefaultTexture()}getFontFamily(){return this._getProperty("fontFamily")}getBreakOn(){return this._getProperty("breakOn")}getWhiteSpace(){return this._getProperty("whiteSpace")}getTextAlign(){return this._getProperty("textAlign")}getTextType(){return this._getProperty("textType")}getFontColor(){return this._getProperty("fontColor")}getFontSupersampling(){return this._getProperty("fontSupersampling")}getFontOpacity(){return this._getProperty("fontOpacity")}getFontPXRange(){return this._getProperty("fontPXRange")}getBorderRadius(){return this._getProperty("borderRadius")}getBorderWidth(){return this._getProperty("borderWidth")}getBorderColor(){return this._getProperty("borderColor")}getBorderOpacity(){return this._getProperty("borderOpacity")}getContainer(){return!this.threeOBJ&&this.parent?this.parent.getContainer():this.threeOBJ?this:Lt.container}getParentsNumber(t){return t=t||0,this.parentUI?this.parentUI.getParentsNumber(t+1):t}getBackgroundOpacity(){return!this.backgroundOpacity&&this.backgroundOpacity!==0?Lt.backgroundOpacity:this.backgroundOpacity}getBackgroundColor(){return this.backgroundColor||Lt.backgroundColor}getBackgroundTexture(){return this.backgroundTexture||Lt.getDefaultTexture()}getAlignContent(){return this.alignContent||Lt.alignContent}getAlignItems(){return this.alignItems||Lt.alignItems}getContentDirection(){return this.contentDirection||Lt.contentDirection}getJustifyContent(){return this.justifyContent||Lt.justifyContent}getInterLine(){return this.interLine===void 0?Lt.interLine:this.interLine}getOffset(){return this.offset===void 0?Lt.offset:this.offset}getBackgroundSize(){return this.backgroundSize===void 0?Lt.backgroundSize:this.backgroundSize}getHiddenOverflow(){return this.hiddenOverflow===void 0?Lt.hiddenOverflow:this.hiddenOverflow}getBestFit(){return this.bestFit===void 0?Lt.bestFit:this.bestFit}_rebuildChildrenLists(){this.childrenUIs=this.children.filter(t=>t.isUI),this.childrenBoxes=this.children.filter(t=>t.isBoxComponent),this.childrenInlines=this.children.filter(t=>t.isInline),this.childrenTexts=this.children.filter(t=>t.isText)}_rebuildParentUI=()=>{this.parent&&this.parent.isUI?this.parentUI=this.parent:this.parentUI=null};add(){for(let i of Object.keys(arguments))arguments[i].isInline&&this.update(null,!0);let t=super.add(...arguments);return this._rebuildChildrenLists(),t}remove(){for(let i of Object.keys(arguments))arguments[i].isInline&&this.update(null,!0);let t=super.remove(...arguments);return this._rebuildChildrenLists(),t}update(t,i,r){mi.requestUpdate(this,t,i,r)}onAfterUpdate(){}_updateFontFamily(t){this.fontFamily=t,this.traverse(i=>{i.isUI&&i.update(!0,!0,!1)}),this.getHighestParent().update(!1,!0,!1)}_updateFontTexture(t){this.fontTexture=t,this.getHighestParent().update(!1,!0,!1)}set(t){let i,r,s;if(mi.register(this),!(!t||JSON.stringify(t)===JSON.stringify({}))){t.alignContent&&(t.alignItems=t.alignContent,t.textAlign||(t.textAlign=t.alignContent),console.warn("`alignContent` property has been deprecated, please rely on `alignItems` and `textAlign` instead."),delete t.alignContent),t.alignItems&&Kv(t.alignItems);for(let o of Object.keys(t))if(this[o]!=t[o])switch(o){case"content":case"fontSize":case"fontKerning":case"breakOn":case"whiteSpace":this.isText&&(i=!0),r=!0,this[o]=t[o];break;case"bestFit":this.isBlock&&(i=!0,r=!0),this[o]=t[o];break;case"width":case"height":case"padding":(this.isInlineBlock||this.isBlock&&this.getBestFit()!="none")&&(i=!0),r=!0,this[o]=t[o];break;case"letterSpacing":case"interLine":this.isBlock&&this.getBestFit()!="none"&&(i=!0),r=!0,this[o]=t[o];break;case"margin":case"contentDirection":case"justifyContent":case"alignContent":case"alignItems":case"textAlign":case"textType":r=!0,this[o]=t[o];break;case"fontColor":case"fontOpacity":case"fontSupersampling":case"offset":case"backgroundColor":case"backgroundOpacity":case"backgroundTexture":case"backgroundSize":case"borderRadius":case"borderWidth":case"borderColor":case"borderOpacity":s=!0,this[o]=t[o];break;case"hiddenOverflow":this[o]=t[o];break}t.fontFamily&&da.setFontFamily(this,t.fontFamily),t.fontTexture&&da.setFontTexture(this,t.fontTexture),this.parentUI&&this.parentUI.getBestFit()!="none"&&this.parentUI.update(!0,!0,!1),this.update(i,r,s),r&&this.getHighestParent().update(!1,!0,!1)}}setupState(t){this.states[t.state]={attributes:t.attributes,onSet:t.onSet}}setState(t){let i=this.states[t];if(!i){console.warn(`state "${t}" does not exist within this component:`,this.name);return}t!==this.currentState&&(this.currentState=t,i.onSet&&i.onSet(),i.attributes&&this.set(i.attributes))}clear(){this.traverse(t=>{mi.disposeOf(t),t.material&&t.material.dispose(),t.geometry&&t.geometry.dispose()})}}}function Hp(n){return class extends n{constructor(t){super(t),this.textUniforms={u_texture:{value:this.getFontTexture()},u_color:{value:this.getFontColor()},u_opacity:{value:this.getFontOpacity()},u_pxRange:{value:this.getFontPXRange()},u_useRGSS:{value:this.getFontSupersampling()}},this.backgroundUniforms={u_texture:{value:this.getBackgroundTexture()},u_color:{value:this.getBackgroundColor()},u_opacity:{value:this.getBackgroundOpacity()},u_backgroundMapping:{value:this.getBackgroundSize()},u_borderWidth:{value:this.getBorderWidth()},u_borderColor:{value:this.getBorderColor()},u_borderRadiusTopLeft:{value:this.getBorderRadius()},u_borderRadiusTopRight:{value:this.getBorderRadius()},u_borderRadiusBottomRight:{value:this.getBorderRadius()},u_borderRadiusBottomLeft:{value:this.getBorderRadius()},u_borderOpacity:{value:this.getBorderOpacity()},u_size:{value:new ke.Vector2(1,1)},u_tSize:{value:new ke.Vector2(1,1)}}}updateBackgroundMaterial(){this.backgroundUniforms.u_texture.value=this.getBackgroundTexture(),this.backgroundUniforms.u_tSize.value.set(this.backgroundUniforms.u_texture.value.image.width,this.backgroundUniforms.u_texture.value.image.height),this.size&&this.backgroundUniforms.u_size.value.copy(this.size),this.backgroundUniforms.u_texture.value.isDefault?(this.backgroundUniforms.u_color.value=this.getBackgroundColor(),this.backgroundUniforms.u_opacity.value=this.getBackgroundOpacity()):(this.backgroundUniforms.u_color.value=this.backgroundColor||Lt.backgroundWhiteColor,this.backgroundUniforms.u_opacity.value=!this.backgroundOpacity&&this.backgroundOpacity!==0?Lt.backgroundOpaqueOpacity:this.backgroundOpacity),this.backgroundUniforms.u_backgroundMapping.value=(()=>{switch(this.getBackgroundSize()){case"stretch":return 0;case"contain":return 1;case"cover":return 2}})();let t=this.getBorderRadius();this.backgroundUniforms.u_borderWidth.value=this.getBorderWidth(),this.backgroundUniforms.u_borderColor.value=this.getBorderColor(),this.backgroundUniforms.u_borderOpacity.value=this.getBorderOpacity(),Array.isArray(t)?(this.backgroundUniforms.u_borderRadiusTopLeft.value=t[0],this.backgroundUniforms.u_borderRadiusTopRight.value=t[1],this.backgroundUniforms.u_borderRadiusBottomRight.value=t[2],this.backgroundUniforms.u_borderRadiusBottomLeft.value=t[3]):(this.backgroundUniforms.u_borderRadiusTopLeft.value=t,this.backgroundUniforms.u_borderRadiusTopRight.value=t,this.backgroundUniforms.u_borderRadiusBottomRight.value=t,this.backgroundUniforms.u_borderRadiusBottomLeft.value=t)}updateTextMaterial(){this.textUniforms.u_texture.value=this.getFontTexture(),this.textUniforms.u_color.value=this.getFontColor(),this.textUniforms.u_opacity.value=this.getFontOpacity(),this.textUniforms.u_pxRange.value=this.getFontPXRange(),this.textUniforms.u_useRGSS.value=this.getFontSupersampling()}getBackgroundMaterial(){return(!this.backgroundMaterial||!this.backgroundUniforms)&&(this.backgroundMaterial=this._makeBackgroundMaterial()),this.backgroundMaterial}getFontMaterial(){return(!this.fontMaterial||!this.textUniforms)&&(this.fontMaterial=this._makeTextMaterial()),this.fontMaterial}_makeTextMaterial(){return new ke.ShaderMaterial({uniforms:this.textUniforms,transparent:!0,clipping:!0,vertexShader:KT,fragmentShader:jT,extensions:{derivatives:!0}})}_makeBackgroundMaterial(){return new ke.ShaderMaterial({uniforms:this.backgroundUniforms,transparent:!0,clipping:!0,vertexShader:QT,fragmentShader:eb,extensions:{derivatives:!0}})}updateClippingPlanes(t){let i=t!==void 0?t:this.getClippingPlanes();JSON.stringify(i)!==JSON.stringify(this.clippingPlanes)&&(this.clippingPlanes=i,this.fontMaterial&&(this.fontMaterial.clippingPlanes=this.clippingPlanes),this.backgroundMaterial&&(this.backgroundMaterial.clippingPlanes=this.clippingPlanes))}}}var KT=`
varying vec2 vUv;

#include <clipping_planes_pars_vertex>

void main() {

	vUv = uv;
	vec4 mvPosition = modelViewMatrix * vec4( position, 1.0 );
	gl_Position = projectionMatrix * mvPosition;
	gl_Position.z -= 0.00001;

	#include <clipping_planes_vertex>

}
`,jT=`

uniform sampler2D u_texture;
uniform vec3 u_color;
uniform float u_opacity;
uniform float u_pxRange;
uniform bool u_useRGSS;

varying vec2 vUv;

#include <clipping_planes_pars_fragment>

// functions from the original msdf repo:
// https://github.com/Chlumsky/msdfgen#using-a-multi-channel-distance-field

float median(float r, float g, float b) {
	return max(min(r, g), min(max(r, g), b));
}

float screenPxRange() {
	vec2 unitRange = vec2(u_pxRange)/vec2(textureSize(u_texture, 0));
	vec2 screenTexSize = vec2(1.0)/fwidth(vUv);
	return max(0.5*dot(unitRange, screenTexSize), 1.0);
}

float tap(vec2 offsetUV) {
	vec3 msd = texture( u_texture, offsetUV ).rgb;
	float sd = median(msd.r, msd.g, msd.b);
	float screenPxDistance = screenPxRange() * (sd - 0.5);
	float alpha = clamp(screenPxDistance + 0.5, 0.0, 1.0);
	return alpha;
}

void main() {

	float alpha;

	if ( u_useRGSS ) {

		// shader-based supersampling based on https://bgolus.medium.com/sharper-mipmapping-using-shader-based-supersampling-ed7aadb47bec
		// per pixel partial derivatives
		vec2 dx = dFdx(vUv);
		vec2 dy = dFdy(vUv);

		// rotated grid uv offsets
		vec2 uvOffsets = vec2(0.125, 0.375);
		vec2 offsetUV = vec2(0.0, 0.0);

		// supersampled using 2x2 rotated grid
		alpha = 0.0;
		offsetUV.xy = vUv + uvOffsets.x * dx + uvOffsets.y * dy;
		alpha += tap(offsetUV);
		offsetUV.xy = vUv - uvOffsets.x * dx - uvOffsets.y * dy;
		alpha += tap(offsetUV);
		offsetUV.xy = vUv + uvOffsets.y * dx - uvOffsets.x * dy;
		alpha += tap(offsetUV);
		offsetUV.xy = vUv - uvOffsets.y * dx + uvOffsets.x * dy;
		alpha += tap(offsetUV);
		alpha *= 0.25;

	} else {

		alpha = tap( vUv );

	}


	// apply the opacity
	alpha *= u_opacity;

	// this is useful to avoid z-fighting when quads overlap because of kerning
	if ( alpha < 0.02) discard;


	gl_FragColor = vec4( u_color, alpha );

	#include <clipping_planes_fragment>

}
`,QT=`
varying vec2 vUv;

#include <clipping_planes_pars_vertex>

void main() {

	vUv = uv;
	vec4 mvPosition = modelViewMatrix * vec4( position, 1.0 );
	gl_Position = projectionMatrix * mvPosition;

	#include <clipping_planes_vertex>

}
`,eb=`

uniform sampler2D u_texture;
uniform vec3 u_color;
uniform float u_opacity;

uniform float u_borderRadiusTopLeft;
uniform float u_borderRadiusTopRight;
uniform float u_borderRadiusBottomLeft;
uniform float u_borderRadiusBottomRight;
uniform float u_borderWidth;
uniform vec3 u_borderColor;
uniform float u_borderOpacity;
uniform vec2 u_size;
uniform vec2 u_tSize;
uniform int u_backgroundMapping;

varying vec2 vUv;

#include <clipping_planes_pars_fragment>

float getEdgeDist() {
	vec2 ndc = vec2( vUv.x * 2.0 - 1.0, vUv.y * 2.0 - 1.0 );
	vec2 planeSpaceCoord = vec2( u_size.x * 0.5 * ndc.x, u_size.y * 0.5 * ndc.y );
	vec2 corner = u_size * 0.5;
	vec2 offsetCorner = corner - abs( planeSpaceCoord );
	float innerRadDist = min( offsetCorner.x, offsetCorner.y ) * -1.0;
	if (vUv.x < 0.5 && vUv.y >= 0.5) {
		float roundedDist = length( max( abs( planeSpaceCoord ) - u_size * 0.5 + u_borderRadiusTopLeft, 0.0 ) ) - u_borderRadiusTopLeft;
		float s = step( innerRadDist * -1.0, u_borderRadiusTopLeft );
		return mix( innerRadDist, roundedDist, s );
	}
	if (vUv.x >= 0.5 && vUv.y >= 0.5) {
		float roundedDist = length( max( abs( planeSpaceCoord ) - u_size * 0.5 + u_borderRadiusTopRight, 0.0 ) ) - u_borderRadiusTopRight;
		float s = step( innerRadDist * -1.0, u_borderRadiusTopRight );
		return mix( innerRadDist, roundedDist, s );
	}
	if (vUv.x >= 0.5 && vUv.y < 0.5) {
		float roundedDist = length( max( abs( planeSpaceCoord ) - u_size * 0.5 + u_borderRadiusBottomRight, 0.0 ) ) - u_borderRadiusBottomRight;
		float s = step( innerRadDist * -1.0, u_borderRadiusBottomRight );
		return mix( innerRadDist, roundedDist, s );
	}
	if (vUv.x < 0.5 && vUv.y < 0.5) {
		float roundedDist = length( max( abs( planeSpaceCoord ) - u_size * 0.5 + u_borderRadiusBottomLeft, 0.0 ) ) - u_borderRadiusBottomLeft;
		float s = step( innerRadDist * -1.0, u_borderRadiusBottomLeft );
		return mix( innerRadDist, roundedDist, s );
	}
}

vec4 sampleTexture() {
	float textureRatio = u_tSize.x / u_tSize.y;
	float panelRatio = u_size.x / u_size.y;
	vec2 uv = vUv;
	if ( u_backgroundMapping == 1 ) { // contain
		if ( textureRatio < panelRatio ) { // repeat on X
			float newX = uv.x * ( panelRatio / textureRatio );
			newX += 0.5 - 0.5 * ( panelRatio / textureRatio );
			uv.x = newX;
		} else { // repeat on Y
			float newY = uv.y * ( textureRatio / panelRatio );
			newY += 0.5 - 0.5 * ( textureRatio / panelRatio );
			uv.y = newY;
		}
	} else if ( u_backgroundMapping == 2 ) { // cover
		if ( textureRatio < panelRatio ) { // stretch on Y
			float newY = uv.y * ( textureRatio / panelRatio );
			newY += 0.5 - 0.5 * ( textureRatio / panelRatio );
			uv.y = newY;
		} else { // stretch on X
			float newX = uv.x * ( panelRatio / textureRatio );
			newX += 0.5 - 0.5 * ( panelRatio / textureRatio );
			uv.x = newX;
		}
	}
	return texture2D( u_texture, uv ).rgba;
}

void main() {

	float edgeDist = getEdgeDist();
	float change = fwidth( edgeDist );

	vec4 textureSample = sampleTexture();
	vec3 blendedColor = textureSample.rgb * u_color;

	float alpha = smoothstep( change, 0.0, edgeDist );
	float blendedOpacity = u_opacity * textureSample.a * alpha;

	vec4 frameColor = vec4( blendedColor, blendedOpacity );

	if ( u_borderWidth <= 0.0 ) {
		gl_FragColor = frameColor;
	} else {
		vec4 borderColor = vec4( u_borderColor, u_borderOpacity * alpha );
		float stp = smoothstep( edgeDist + change, edgeDist, u_borderWidth * -1.0 );
		gl_FragColor = mix( frameColor, borderColor, stp );
	}

	#include <clipping_planes_fragment>
}
`,Hc=class extends ke.Mesh{constructor(e){let t=new ke.PlaneGeometry;super(t,e),this.castShadow=!0,this.receiveShadow=!0,this.name="MeshUI-Frame"}},zc=null;function Xs(...n){if(!zc)throw new Error("Cannot use mixins with Base null");let e=zc;zc=null;let t=n.length,i;for(;--t>=0;)i=n[t],e=i(e);return e}Xs.withBase=n=>(zc=n,Xs);var Ur=class extends Xs.withBase(ke.Object3D)(Bp,cy,Hp,jc){constructor(e){super(e),this.isBlock=!0,this.size=new ke.Vector2(1,1),this.frame=new Hc(this.getBackgroundMaterial()),this.frame.onBeforeRender=()=>{this.updateClippingPlanes&&this.updateClippingPlanes()},this.add(this.frame),this.set(e)}parseParams(){let e=this.getBestFit();e!="none"&&this.childrenTexts.length?this.calculateBestFit(e):this.childrenTexts.forEach(t=>{t._fitFontSize=void 0})}updateLayout(){let e=this.getWidth(),t=this.getHeight();if(!e||!t){console.warn("Block got no dimension from its parameters or from children parameters");return}this.size.set(e,t),this.frame.scale.set(e,t,1),this.frame&&this.updateBackgroundMaterial(),this.frame.renderOrder=this.getParentsNumber(),this.autoLayout&&this.setPosFromParentRecords(),this.childrenInlines.length&&this.computeInlinesPosition(),this.computeChildrenPosition(),this.parentUI&&(this.position.z=this.getOffset())}updateInner(){this.parentUI&&(this.position.z=this.getOffset()),this.frame&&this.updateBackgroundMaterial()}};function hy(n){return class extends n{constructor(t){super(t),this.isInline=!0}}}function tb(n,e=!1){let t=n[0].index!==null,i=new Set(Object.keys(n[0].attributes)),r=new Set(Object.keys(n[0].morphAttributes)),s={},o={},a=n[0].morphTargetsRelative,l=new ke.BufferGeometry,c=0;for(let u=0;u<n.length;++u){let p=n[u],f=0;if(t!==(p.index!==null))return console.error("THREE.BufferGeometryUtils: .mergeBufferGeometries() failed with geometry at index "+u+". All geometries must have compatible attributes; make sure index attribute exists among all geometries, or in none of them."),null;for(let m in p.attributes){if(!i.has(m))return console.error("THREE.BufferGeometryUtils: .mergeBufferGeometries() failed with geometry at index "+u+'. All geometries must have compatible attributes; make sure "'+m+'" attribute exists among all geometries, or in none of them.'),null;s[m]===void 0&&(s[m]=[]),s[m].push(p.attributes[m]),f++}if(f!==i.size)return console.error("THREE.BufferGeometryUtils: .mergeBufferGeometries() failed with geometry at index "+u+". Make sure all geometries have the same number of attributes."),null;if(a!==p.morphTargetsRelative)return console.error("THREE.BufferGeometryUtils: .mergeBufferGeometries() failed with geometry at index "+u+". .morphTargetsRelative must be consistent throughout all geometries."),null;for(let m in p.morphAttributes){if(!r.has(m))return console.error("THREE.BufferGeometryUtils: .mergeBufferGeometries() failed with geometry at index "+u+".  .morphAttributes must be consistent throughout all geometries."),null;o[m]===void 0&&(o[m]=[]),o[m].push(p.morphAttributes[m])}if(l.userData.mergedUserData=l.userData.mergedUserData||[],l.userData.mergedUserData.push(p.userData),e){let m;if(t)m=p.index.count;else if(p.attributes.position!==void 0)m=p.attributes.position.count;else return console.error("THREE.BufferGeometryUtils: .mergeBufferGeometries() failed with geometry at index "+u+". The geometry must have either an index or a position attribute"),null;l.addGroup(c,m,u),c+=m}}if(t){let u=0,p=[];for(let f=0;f<n.length;++f){let m=n[f].index;for(let _=0;_<m.count;++_)p.push(m.getX(_)+u);u+=n[f].attributes.position.count}l.setIndex(p)}for(let u in s){let p=Xv(s[u]);if(!p)return console.error("THREE.BufferGeometryUtils: .mergeBufferGeometries() failed while trying to merge the "+u+" attribute."),null;l.setAttribute(u,p)}for(let u in o){let p=o[u][0].length;if(p===0)break;l.morphAttributes=l.morphAttributes||{},l.morphAttributes[u]=[];for(let f=0;f<p;++f){let m=[];for(let y=0;y<o[u].length;++y)m.push(o[u][y][f]);let _=Xv(m);if(!_)return console.error("THREE.BufferGeometryUtils: .mergeBufferGeometries() failed while trying to merge the "+u+" morphAttribute."),null;l.morphAttributes[u].push(_)}}return l}function Xv(n){let e,t,i,r=0;for(let a=0;a<n.length;++a){let l=n[a];if(l.isInterleavedBufferAttribute)return console.error("THREE.BufferGeometryUtils: .mergeBufferAttributes() failed. InterleavedBufferAttributes are not supported."),null;if(e===void 0&&(e=l.array.constructor),e!==l.array.constructor)return console.error("THREE.BufferGeometryUtils: .mergeBufferAttributes() failed. BufferAttribute.array must be of consistent array types across matching attributes."),null;if(t===void 0&&(t=l.itemSize),t!==l.itemSize)return console.error("THREE.BufferGeometryUtils: .mergeBufferAttributes() failed. BufferAttribute.itemSize must be consistent across matching attributes."),null;if(i===void 0&&(i=l.normalized),i!==l.normalized)return console.error("THREE.BufferGeometryUtils: .mergeBufferAttributes() failed. BufferAttribute.normalized must be consistent across matching attributes."),null;r+=l.array.length}let s=new e(r),o=0;for(let a=0;a<n.length;++a)s.set(n[a].array,o),o+=n[a].array.length;return new ke.BufferAttribute(s,t,i)}var Ap=class extends ke.PlaneGeometry{constructor(e,t){let i=e.glyph,r=e.fontSize;super(e.width,e.height),i.match(/\s/g)===null?(t.info.charset.indexOf(i)===-1&&console.error(`The character '${i}' is not included in the font characters set.`),this.mapUVs(t,i),this.transformGeometry(e)):(this.nullifyUVs(),this.scale(0,0,1),this.translate(0,r/2,0))}mapUVs(e,t){let i=e.chars.find(u=>u.char===t),r=e.common,s=i.x/r.scaleW,o=(i.x+i.width)/r.scaleW,a=1-(i.y+i.height)/r.scaleH,l=1-i.y/r.scaleH,c=this.attributes.uv;for(let u=0;u<c.count;u++){let p=c.getX(u),f=c.getY(u);[p,f]=(()=>{switch(u){case 0:return[s,l];case 1:return[o,l];case 2:return[s,a];case 3:return[o,a]}})(),c.setXY(u,p,f)}}nullifyUVs(){let e=this.attributes.uv;for(let t=0;t<e.count;t++)e.setXY(t,0,0)}transformGeometry(e){this.translate(e.width/2,e.height/2,0)}};function nb(n){let e=n.font,t=n.fontSize,i=n.glyph,r=t/e.info.size,s=e.chars.find(p=>p.char===i),o=s?s.width*r:t/3,a=s?s.height*r:0;o===0&&(o=s?s.xadvance*r:t),a===0&&(a=t*.7),i===`
`&&(o=0);let l=s?s.xadvance*r:o,c=s?s.xoffset*r:0,u=s?s.yoffset*r:0;return{width:o,height:a,anchor:u,xadvance:l,xoffset:c}}function ib(n,e){let t=n._kernings;return t[e]?t[e]:0}function rb(){let n=[];this.inlines.forEach((i,r)=>{n[r]=new Ap(i,this.getFontFamily()),n[r].translate(i.offsetX,i.offsetY,0)});let e=tb(n);return new ke.Mesh(e,this.getFontMaterial())}var Ep={getGlyphDimensions:nb,getGlyphPairKerning:ib,buildText:rb};function sb(n){return class extends n{createText(){let t=this,i=(()=>{switch(this.getTextType()){case"MSDF":return Ep.buildText.call(this);default:console.warn(`'${this.getTextType()}' is not a supported text type.
See https://github.com/felixmariotto/three-mesh-ui/wiki/Using-a-custom-text-type`);break}})();return i.renderOrder=1/0,i.onBeforeRender=function(){t.updateClippingPlanes&&t.updateClippingPlanes()},i}getGlyphDimensions(t){switch(t.textType){case"MSDF":return Ep.getGlyphDimensions(t);default:console.warn(`'${t.textType}' is not a supported text type.
See https://github.com/felixmariotto/three-mesh-ui/wiki/Using-a-custom-text-type`);break}}getGlyphPairKerning(t,i,r){switch(t){case"MSDF":return Ep.getGlyphPairKerning(i,r);default:console.warn(`'${t}' is not a supported text type.
See https://github.com/felixmariotto/three-mesh-ui/wiki/Using-a-custom-text-type`);break}}}}function dy(n){n.children.forEach(e=>{e.children.length>0&&dy(e),n.remove(e),mi.disposeOf(e),e.material&&e.material.dispose(),e.geometry&&e.geometry.dispose()}),n.children=[]}var ob=dy,fa=class extends Xs.withBase(ke.Object3D)(hy,sb,Hp,jc){constructor(e){super(e),this.isText=!0,this.set(e)}parseParams(){this.calculateInlines(this._fitFontSize||this.getFontSize())}updateLayout(){ob(this),this.inlines&&(this.textContent=this.createText(),this.updateTextMaterial(),this.add(this.textContent)),this.position.z=this.getOffset()}updateInner(){this.position.z=this.getOffset(),this.textContent&&this.updateTextMaterial()}calculateInlines(e){let t=this.content,i=this.getFontFamily(),r=this.getBreakOn(),s=this.getTextType(),o=this.getWhiteSpace();if(!i||typeof i=="string"){da.getFontOf(this)||console.warn("no font was found");return}if(!this.content){this.inlines=null;return}if(!s){console.error(`You must provide a 'textType' attribute so three-mesh-ui knows how to render your text.
 See https://github.com/felixmariotto/three-mesh-ui/wiki/Using-a-custom-text-type`);return}let a=Qv(t,o),l=Array.from?Array.from(a):String(a).split(""),c=e/i.info.size,u=i.common.lineHeight*c,p=i.common.base*c,f=l.map(m=>{let _=this.getGlyphDimensions({textType:s,glyph:m,font:i,fontSize:e}),y=null;return o!==Ys&&(r.includes(m)||m.match(/\s/g))&&(y="possible"),m.match(/\n/g)&&(y=ey(o)),{height:_.height,width:_.width,anchor:_.anchor,xadvance:_.xadvance,xoffset:_.xoffset,lineBreak:y,glyph:m,fontSize:e,lineHeight:u,lineBase:p}});if(this.getFontKerning()!=="none")for(let m=1;m<f.length;m++){let _=f[m],y=f[m-1].glyph+f[m].glyph,d=this.getGlyphPairKerning(s,i,y);_.kerning=d*(e/i.info.size)}this.inlines=f}},pa=class extends Xs.withBase(ke.Object3D)(hy,Bp,cy,Hp,jc){constructor(e){super(e),this.isInlineBlock=!0,this.size=new ke.Vector2(1,1),this.frame=new Hc(this.getBackgroundMaterial()),this.frame.onBeforeRender=()=>{this.updateClippingPlanes&&this.updateClippingPlanes()},this.add(this.frame),this.set(e)}parseParams(){this.width||console.warn("inlineBlock has no width. Set to 0.3 by default"),this.height||console.warn("inlineBlock has no height. Set to 0.3 by default"),this.inlines=[{height:this.height||.3,width:this.width||.3,anchor:0,lineBreak:"possible"}]}updateLayout(){let e=this.getWidth(),t=this.getHeight();if(this.inlines){let i=this.inlines[0];this.position.set(i.width/2,i.height/2,0),this.position.x+=i.offsetX,this.position.y+=i.offsetY}this.size.set(e,t),this.frame.scale.set(e,t,1),this.frame&&this.updateBackgroundMaterial(),this.frame.renderOrder=this.getParentsNumber(),this.childrenInlines.length&&this.computeInlinesPosition(),this.computeChildrenPosition(),this.position.z=this.getOffset()}updateInner(){this.position.z=this.getOffset(),this.frame&&this.updateBackgroundMaterial()}},Ki={fr:[[[{width:.1,chars:[{lowerCase:"a",upperCase:"A"}]},{width:.1,chars:[{lowerCase:"z",upperCase:"Z"}]},{width:.1,chars:[{lowerCase:"e",upperCase:"E"}]},{width:.1,chars:[{lowerCase:"r",upperCase:"R"}]},{width:.1,chars:[{lowerCase:"t",upperCase:"T"}]},{width:.1,chars:[{lowerCase:"y",upperCase:"Y"}]},{width:.1,chars:[{lowerCase:"u",upperCase:"U"}]},{width:.1,chars:[{lowerCase:"i",upperCase:"I"}]},{width:.1,chars:[{lowerCase:"o",upperCase:"O"}]},{width:.1,chars:[{lowerCase:"p",upperCase:"P"}]}],[{width:.1,chars:[{lowerCase:"q",upperCase:"Q"}]},{width:.1,chars:[{lowerCase:"s",upperCase:"S"}]},{width:.1,chars:[{lowerCase:"d",upperCase:"D"}]},{width:.1,chars:[{lowerCase:"f",upperCase:"F"}]},{width:.1,chars:[{lowerCase:"g",upperCase:"G"}]},{width:.1,chars:[{lowerCase:"h",upperCase:"H"}]},{width:.1,chars:[{lowerCase:"j",upperCase:"J"}]},{width:.1,chars:[{lowerCase:"k",upperCase:"K"}]},{width:.1,chars:[{lowerCase:"l",upperCase:"L"}]},{width:.1,chars:[{lowerCase:"m",upperCase:"M"}]}],[{width:.2,command:"shift",chars:[{icon:"shift"}]},{width:.1,chars:[{lowerCase:"w",upperCase:"W"}]},{width:.1,chars:[{lowerCase:"x",upperCase:"X"}]},{width:.1,chars:[{lowerCase:"c",upperCase:"C"}]},{width:.1,chars:[{lowerCase:"v",upperCase:"V"}]},{width:.1,chars:[{lowerCase:"b",upperCase:"B"}]},{width:.1,chars:[{lowerCase:"n",upperCase:"N"}]},{width:.2,command:"backspace",chars:[{icon:"backspace"}]}],[{width:.2,command:"switch",chars:[{lowerCase:".?12"}]},{width:.1,chars:[{lowerCase:","}]},{width:.4,command:"space",chars:[{icon:"space"}]},{width:.1,chars:[{lowerCase:"."}]},{width:.2,command:"enter",chars:[{icon:"enter"}]}]],[[{width:.1,chars:[{lowerCase:"1"}]},{width:.1,chars:[{lowerCase:"2"}]},{width:.1,chars:[{lowerCase:"3"}]},{width:.1,chars:[{lowerCase:"4"}]},{width:.1,chars:[{lowerCase:"5"}]},{width:.1,chars:[{lowerCase:"6"}]},{width:.1,chars:[{lowerCase:"7"}]},{width:.1,chars:[{lowerCase:"8"}]},{width:.1,chars:[{lowerCase:"9"}]},{width:.1,chars:[{lowerCase:"0"}]}],[{width:.1,chars:[{lowerCase:"@"}]},{width:.1,chars:[{lowerCase:"#"}]},{width:.1,chars:[{lowerCase:"|"}]},{width:.1,chars:[{lowerCase:"_"}]},{width:.1,chars:[{lowerCase:"&"}]},{width:.1,chars:[{lowerCase:"-"}]},{width:.1,chars:[{lowerCase:"+"}]},{width:.1,chars:[{lowerCase:"("}]},{width:.1,chars:[{lowerCase:")"}]},{width:.1,chars:[{lowerCase:"/"}]}],[{width:.1,chars:[{lowerCase:"="}]},{width:.1,chars:[{lowerCase:"*"}]},{width:.1,chars:[{lowerCase:'"'}]},{width:.1,chars:[{lowerCase:"'"}]},{width:.1,chars:[{lowerCase:":"}]},{width:.1,chars:[{lowerCase:";"}]},{width:.1,chars:[{lowerCase:"!"}]},{width:.1,chars:[{lowerCase:"?"}]},{width:.2,command:"backspace",chars:[{icon:"backspace"}]}],[{width:.2,command:"switch",chars:[{lowerCase:".?12"}]},{width:.1,chars:[{lowerCase:","}]},{width:.4,command:"space",chars:[{icon:"space"}]},{width:.1,chars:[{lowerCase:"."}]},{width:.2,command:"enter",chars:[{icon:"enter"}]}]]],eng:[[[{width:.1,chars:[{lowerCase:"q",upperCase:"Q"}]},{width:.1,chars:[{lowerCase:"w",upperCase:"W"}]},{width:.1,chars:[{lowerCase:"e",upperCase:"E"}]},{width:.1,chars:[{lowerCase:"r",upperCase:"R"}]},{width:.1,chars:[{lowerCase:"t",upperCase:"T"}]},{width:.1,chars:[{lowerCase:"y",upperCase:"Y"}]},{width:.1,chars:[{lowerCase:"u",upperCase:"U"}]},{width:.1,chars:[{lowerCase:"i",upperCase:"I"}]},{width:.1,chars:[{lowerCase:"o",upperCase:"O"}]},{width:.1,chars:[{lowerCase:"p",upperCase:"P"}]}],[{width:.1,chars:[{lowerCase:"a",upperCase:"A"}]},{width:.1,chars:[{lowerCase:"s",upperCase:"S"}]},{width:.1,chars:[{lowerCase:"d",upperCase:"D"}]},{width:.1,chars:[{lowerCase:"f",upperCase:"F"}]},{width:.1,chars:[{lowerCase:"g",upperCase:"G"}]},{width:.1,chars:[{lowerCase:"h",upperCase:"H"}]},{width:.1,chars:[{lowerCase:"j",upperCase:"J"}]},{width:.1,chars:[{lowerCase:"k",upperCase:"K"}]},{width:.1,chars:[{lowerCase:"l",upperCase:"L"}]}],[{width:.15,command:"shift",chars:[{icon:"shift"}]},{width:.1,chars:[{lowerCase:"z",upperCase:"Z"}]},{width:.1,chars:[{lowerCase:"x",upperCase:"X"}]},{width:.1,chars:[{lowerCase:"c",upperCase:"C"}]},{width:.1,chars:[{lowerCase:"v",upperCase:"V"}]},{width:.1,chars:[{lowerCase:"b",upperCase:"B"}]},{width:.1,chars:[{lowerCase:"n",upperCase:"N"}]},{width:.1,chars:[{lowerCase:"m",upperCase:"M"}]},{width:.15,command:"backspace",chars:[{icon:"backspace"}]}],[{width:.2,command:"switch",chars:[{lowerCase:".?12"}]},{width:.1,chars:[{lowerCase:","}]},{width:.4,command:"space",chars:[{icon:"space"}]},{width:.1,chars:[{lowerCase:"."}]},{width:.2,command:"enter",chars:[{icon:"enter"}]}]],[[{width:.1,chars:[{lowerCase:"1"}]},{width:.1,chars:[{lowerCase:"2"}]},{width:.1,chars:[{lowerCase:"3"}]},{width:.1,chars:[{lowerCase:"4"}]},{width:.1,chars:[{lowerCase:"5"}]},{width:.1,chars:[{lowerCase:"6"}]},{width:.1,chars:[{lowerCase:"7"}]},{width:.1,chars:[{lowerCase:"8"}]},{width:.1,chars:[{lowerCase:"9"}]},{width:.1,chars:[{lowerCase:"0"}]}],[{width:.1,chars:[{lowerCase:"@"}]},{width:.1,chars:[{lowerCase:"#"}]},{width:.1,chars:[{lowerCase:"|"}]},{width:.1,chars:[{lowerCase:"_"}]},{width:.1,chars:[{lowerCase:"&"}]},{width:.1,chars:[{lowerCase:"-"}]},{width:.1,chars:[{lowerCase:"+"}]},{width:.1,chars:[{lowerCase:"("}]},{width:.1,chars:[{lowerCase:")"}]},{width:.1,chars:[{lowerCase:"/"}]}],[{width:.1,chars:[{lowerCase:"="}]},{width:.1,chars:[{lowerCase:"*"}]},{width:.1,chars:[{lowerCase:'"'}]},{width:.1,chars:[{lowerCase:"'"}]},{width:.1,chars:[{lowerCase:":"}]},{width:.1,chars:[{lowerCase:";"}]},{width:.1,chars:[{lowerCase:"!"}]},{width:.1,chars:[{lowerCase:"?"}]},{width:.2,command:"backspace",chars:[{icon:"backspace"}]}],[{width:.2,command:"switch",chars:[{lowerCase:".?12"}]},{width:.1,chars:[{lowerCase:","}]},{width:.4,command:"space",chars:[{icon:"space"}]},{width:.1,chars:[{lowerCase:"."}]},{width:.2,command:"enter",chars:[{icon:"enter"}]}]]],ru:[[[{width:1/12,chars:[{lowerCase:"\u0439",upperCase:"\u0419"},{lowerCase:"q",upperCase:"Q"}]},{width:1/12,chars:[{lowerCase:"\u0446",upperCase:"\u0426"},{lowerCase:"w",upperCase:"W"}]},{width:1/12,chars:[{lowerCase:"\u0443",upperCase:"\u0423"},{lowerCase:"e",upperCase:"E"}]},{width:1/12,chars:[{lowerCase:"\u043A",upperCase:"\u041A"},{lowerCase:"r",upperCase:"R"}]},{width:1/12,chars:[{lowerCase:"\u0435",upperCase:"\u0415"},{lowerCase:"t",upperCase:"T"}]},{width:1/12,chars:[{lowerCase:"\u043D",upperCase:"\u041D"},{lowerCase:"y",upperCase:"Y"}]},{width:1/12,chars:[{lowerCase:"\u0433",upperCase:"\u0413"},{lowerCase:"u",upperCase:"U"}]},{width:1/12,chars:[{lowerCase:"\u0448",upperCase:"\u0428"},{lowerCase:"i",upperCase:"I"}]},{width:1/12,chars:[{lowerCase:"\u0449",upperCase:"\u0429"},{lowerCase:"o",upperCase:"O"}]},{width:1/12,chars:[{lowerCase:"\u0437",upperCase:"\u0417"},{lowerCase:"p",upperCase:"P"}]},{width:1/12,chars:[{lowerCase:"\u0445",upperCase:"\u0425"},{lowerCase:"{",upperCase:"["}]},{width:1/12,chars:[{lowerCase:"\u044A",upperCase:"\u042A"},{lowerCase:"}",upperCase:"]"}]}],[{width:1/12,chars:[{lowerCase:"\u0444",upperCase:"\u0424"},{lowerCase:"a",upperCase:"A"}]},{width:1/12,chars:[{lowerCase:"\u044B",upperCase:"\u042B"},{lowerCase:"s",upperCase:"S"}]},{width:1/12,chars:[{lowerCase:"\u0432",upperCase:"\u0412"},{lowerCase:"d",upperCase:"D"}]},{width:1/12,chars:[{lowerCase:"\u0430",upperCase:"\u0410"},{lowerCase:"f",upperCase:"F"}]},{width:1/12,chars:[{lowerCase:"\u043F",upperCase:"\u041F"},{lowerCase:"g",upperCase:"G"}]},{width:1/12,chars:[{lowerCase:"\u0440",upperCase:"\u0420"},{lowerCase:"h",upperCase:"H"}]},{width:1/12,chars:[{lowerCase:"\u043E",upperCase:"\u041E"},{lowerCase:"j",upperCase:"J"}]},{width:1/12,chars:[{lowerCase:"\u043B",upperCase:"\u041B"},{lowerCase:"k",upperCase:"K"}]},{width:1/12,chars:[{lowerCase:"\u0434",upperCase:"\u0414"},{lowerCase:"l",upperCase:"L"}]},{width:1/12,chars:[{lowerCase:"\u0436",upperCase:"\u0416"},{lowerCase:":",upperCase:";"}]},{width:1/12,chars:[{lowerCase:"\u044D",upperCase:"\u042D"},{lowerCase:'"',upperCase:"'"}]},{width:1/12,chars:[{lowerCase:"\u0451",upperCase:"\u0401"},{lowerCase:"|",upperCase:"\\"}]}],[{width:1.5/12,command:"shift",chars:[{icon:"shift"}]},{width:1/12,chars:[{lowerCase:"\u044F",upperCase:"\u042F"},{lowerCase:"z",upperCase:"Z"}]},{width:1/12,chars:[{lowerCase:"\u0447",upperCase:"\u0427"},{lowerCase:"x",upperCase:"X"}]},{width:1/12,chars:[{lowerCase:"\u0441",upperCase:"\u0421"},{lowerCase:"c",upperCase:"C"}]},{width:1/12,chars:[{lowerCase:"\u043C",upperCase:"\u041C"},{lowerCase:"v",upperCase:"V"}]},{width:1/12,chars:[{lowerCase:"\u0438",upperCase:"\u0418"},{lowerCase:"b",upperCase:"B"}]},{width:1/12,chars:[{lowerCase:"\u0442",upperCase:"\u0422"},{lowerCase:"n",upperCase:"N"}]},{width:1/12,chars:[{lowerCase:"\u044C",upperCase:"\u042C"},{lowerCase:"m",upperCase:"M"}]},{width:1/12,chars:[{lowerCase:"\u0431",upperCase:"\u0411"},{lowerCase:",",upperCase:""}]},{width:1/12,chars:[{lowerCase:"\u044E",upperCase:"\u042E"},{lowerCase:".",upperCase:""}]},{width:1.5/12,command:"backspace",chars:[{icon:"backspace"}]}],[{width:.15,command:"switch-set",chars:[{lowerCase:"eng"}]},{width:.15,command:"switch",chars:[{lowerCase:".?12"}]},{width:.4,command:"space",chars:[{icon:"space"}]},{width:.1,chars:[{lowerCase:"?"}]},{width:.2,command:"enter",chars:[{icon:"enter"}]}]],[[{width:.1,chars:[{lowerCase:"1"}]},{width:.1,chars:[{lowerCase:"2"}]},{width:.1,chars:[{lowerCase:"3"}]},{width:.1,chars:[{lowerCase:"4"}]},{width:.1,chars:[{lowerCase:"5"}]},{width:.1,chars:[{lowerCase:"6"}]},{width:.1,chars:[{lowerCase:"7"}]},{width:.1,chars:[{lowerCase:"8"}]},{width:.1,chars:[{lowerCase:"9"}]},{width:.1,chars:[{lowerCase:"0"}]}],[{width:.1,chars:[{lowerCase:"@"}]},{width:.1,chars:[{lowerCase:"#"}]},{width:.1,chars:[{lowerCase:"|"}]},{width:.1,chars:[{lowerCase:"_"}]},{width:.1,chars:[{lowerCase:"&"}]},{width:.1,chars:[{lowerCase:"-"}]},{width:.1,chars:[{lowerCase:"+"}]},{width:.1,chars:[{lowerCase:"("}]},{width:.1,chars:[{lowerCase:")"}]},{width:.1,chars:[{lowerCase:"/"}]}],[{width:.1,chars:[{lowerCase:"="}]},{width:.1,chars:[{lowerCase:"*"}]},{width:.1,chars:[{lowerCase:'"'}]},{width:.1,chars:[{lowerCase:"'"}]},{width:.1,chars:[{lowerCase:":"}]},{width:.1,chars:[{lowerCase:";"}]},{width:.1,chars:[{lowerCase:"!"}]},{width:.1,chars:[{lowerCase:"?"}]},{width:.2,command:"backspace",chars:[{icon:"backspace"}]}],[{width:.3,command:"switch",chars:[{lowerCase:"\u0410\u0411\u0412"}]},{width:.4,command:"space",chars:[{icon:"space"}]},{width:.1,chars:[{lowerCase:"."}]},{width:.2,command:"enter",chars:[{icon:"enter"}]}]]],de:[[[{width:1/11,chars:[{lowerCase:"q",upperCase:"Q"}]},{width:1/11,chars:[{lowerCase:"w",upperCase:"W"}]},{width:1/11,chars:[{lowerCase:"e",upperCase:"E"}]},{width:1/11,chars:[{lowerCase:"r",upperCase:"R"}]},{width:1/11,chars:[{lowerCase:"t",upperCase:"T"}]},{width:1/11,chars:[{lowerCase:"z",upperCase:"Z"}]},{width:1/11,chars:[{lowerCase:"u",upperCase:"U"}]},{width:1/11,chars:[{lowerCase:"i",upperCase:"I"}]},{width:1/11,chars:[{lowerCase:"o",upperCase:"O"}]},{width:1/11,chars:[{lowerCase:"p",upperCase:"P"}]},{width:1/11,chars:[{lowerCase:"\xFC",upperCase:"\xDC"}]}],[{width:1/11,chars:[{lowerCase:"a",upperCase:"A"}]},{width:1/11,chars:[{lowerCase:"s",upperCase:"S"}]},{width:1/11,chars:[{lowerCase:"d",upperCase:"D"}]},{width:1/11,chars:[{lowerCase:"f",upperCase:"F"}]},{width:1/11,chars:[{lowerCase:"g",upperCase:"G"}]},{width:1/11,chars:[{lowerCase:"h",upperCase:"H"}]},{width:1/11,chars:[{lowerCase:"j",upperCase:"J"}]},{width:1/11,chars:[{lowerCase:"k",upperCase:"K"}]},{width:1/11,chars:[{lowerCase:"l",upperCase:"L"}]},{width:1/11,chars:[{lowerCase:"\xF6",upperCase:"\xD6"}]},{width:1/11,chars:[{lowerCase:"\xE4",upperCase:"\xC4"}]}],[{width:2/11,command:"shift",chars:[{icon:"shift"}]},{width:1/11,chars:[{lowerCase:"y",upperCase:"Y"}]},{width:1/11,chars:[{lowerCase:"x",upperCase:"X"}]},{width:1/11,chars:[{lowerCase:"c",upperCase:"C"}]},{width:1/11,chars:[{lowerCase:"v",upperCase:"V"}]},{width:1/11,chars:[{lowerCase:"b",upperCase:"B"}]},{width:1/11,chars:[{lowerCase:"n",upperCase:"N"}]},{width:1/11,chars:[{lowerCase:"m",upperCase:"M"}]},{width:2/11,command:"backspace",chars:[{icon:"backspace"}]}],[{width:.2,command:"switch",chars:[{lowerCase:".?12"}]},{width:.1,chars:[{lowerCase:","}]},{width:.4,command:"space",chars:[{icon:"space"}]},{width:.1,chars:[{lowerCase:"."}]},{width:.2,command:"enter",chars:[{icon:"enter"}]}]],[[{width:.1,chars:[{lowerCase:"1"}]},{width:.1,chars:[{lowerCase:"2"}]},{width:.1,chars:[{lowerCase:"3"}]},{width:.1,chars:[{lowerCase:"4"}]},{width:.1,chars:[{lowerCase:"5"}]},{width:.1,chars:[{lowerCase:"6"}]},{width:.1,chars:[{lowerCase:"7"}]},{width:.1,chars:[{lowerCase:"8"}]},{width:.1,chars:[{lowerCase:"9"}]},{width:.1,chars:[{lowerCase:"0"}]}],[{width:.1,chars:[{lowerCase:"@"}]},{width:.1,chars:[{lowerCase:"#"}]},{width:.1,chars:[{lowerCase:"|"}]},{width:.1,chars:[{lowerCase:"_"}]},{width:.1,chars:[{lowerCase:"&"}]},{width:.1,chars:[{lowerCase:"-"}]},{width:.1,chars:[{lowerCase:"+"}]},{width:.1,chars:[{lowerCase:"("}]},{width:.1,chars:[{lowerCase:")"}]},{width:.1,chars:[{lowerCase:"/"}]}],[{width:.1,chars:[{lowerCase:"="}]},{width:.1,chars:[{lowerCase:"*"}]},{width:.1,chars:[{lowerCase:'"'}]},{width:.1,chars:[{lowerCase:"'"}]},{width:.1,chars:[{lowerCase:":"}]},{width:.1,chars:[{lowerCase:";"}]},{width:.1,chars:[{lowerCase:"!"}]},{width:.1,chars:[{lowerCase:"?"}]},{width:.2,command:"backspace",chars:[{icon:"backspace"}]}],[{width:.2,command:"switch",chars:[{lowerCase:".?12"}]},{width:.1,chars:[{lowerCase:","}]},{width:.4,command:"space",chars:[{icon:"space"}]},{width:.1,chars:[{lowerCase:"."}]},{width:.2,command:"enter",chars:[{icon:"enter"}]}]]],es:[[[{width:.1,chars:[{lowerCase:"q",upperCase:"Q"}]},{width:.1,chars:[{lowerCase:"w",upperCase:"W"}]},{width:.1,chars:[{lowerCase:"e",upperCase:"E"}]},{width:.1,chars:[{lowerCase:"r",upperCase:"R"}]},{width:.1,chars:[{lowerCase:"t",upperCase:"T"}]},{width:.1,chars:[{lowerCase:"y",upperCase:"Y"}]},{width:.1,chars:[{lowerCase:"u",upperCase:"U"}]},{width:.1,chars:[{lowerCase:"i",upperCase:"I"}]},{width:.1,chars:[{lowerCase:"o",upperCase:"O"}]},{width:.1,chars:[{lowerCase:"p",upperCase:"P"}]}],[{width:.1,chars:[{lowerCase:"a",upperCase:"A"}]},{width:.1,chars:[{lowerCase:"s",upperCase:"S"}]},{width:.1,chars:[{lowerCase:"d",upperCase:"D"}]},{width:.1,chars:[{lowerCase:"f",upperCase:"F"}]},{width:.1,chars:[{lowerCase:"g",upperCase:"G"}]},{width:.1,chars:[{lowerCase:"h",upperCase:"H"}]},{width:.1,chars:[{lowerCase:"j",upperCase:"J"}]},{width:.1,chars:[{lowerCase:"k",upperCase:"K"}]},{width:.1,chars:[{lowerCase:"l",upperCase:"L"}]},{width:.1,chars:[{lowerCase:"\xF1",upperCase:"\xD1"}]}],[{width:.15,command:"shift",chars:[{icon:"shift"}]},{width:.1,chars:[{lowerCase:"z",upperCase:"Z"}]},{width:.1,chars:[{lowerCase:"x",upperCase:"X"}]},{width:.1,chars:[{lowerCase:"c",upperCase:"C"}]},{width:.1,chars:[{lowerCase:"v",upperCase:"V"}]},{width:.1,chars:[{lowerCase:"b",upperCase:"B"}]},{width:.1,chars:[{lowerCase:"n",upperCase:"N"}]},{width:.1,chars:[{lowerCase:"m",upperCase:"M"}]},{width:.15,command:"backspace",chars:[{icon:"backspace"}]}],[{width:.2,command:"switch",chars:[{lowerCase:".?12"}]},{width:.1,chars:[{lowerCase:","}]},{width:.4,command:"space",chars:[{icon:"space"}]},{width:.1,chars:[{lowerCase:"."}]},{width:.2,command:"enter",chars:[{icon:"enter"}]}]],[[{width:.1,chars:[{lowerCase:"1"}]},{width:.1,chars:[{lowerCase:"2"}]},{width:.1,chars:[{lowerCase:"3"}]},{width:.1,chars:[{lowerCase:"4"}]},{width:.1,chars:[{lowerCase:"5"}]},{width:.1,chars:[{lowerCase:"6"}]},{width:.1,chars:[{lowerCase:"7"}]},{width:.1,chars:[{lowerCase:"8"}]},{width:.1,chars:[{lowerCase:"9"}]},{width:.1,chars:[{lowerCase:"0"}]}],[{width:.1,chars:[{lowerCase:"@"}]},{width:.1,chars:[{lowerCase:"#"}]},{width:.1,chars:[{lowerCase:"|"}]},{width:.1,chars:[{lowerCase:"_"}]},{width:.1,chars:[{lowerCase:"&"}]},{width:.1,chars:[{lowerCase:"-"}]},{width:.1,chars:[{lowerCase:"+"}]},{width:.1,chars:[{lowerCase:"("}]},{width:.1,chars:[{lowerCase:")"}]},{width:.1,chars:[{lowerCase:"/"}]}],[{width:.1,chars:[{lowerCase:"="}]},{width:.1,chars:[{lowerCase:"*"}]},{width:.1,chars:[{lowerCase:'"'}]},{width:.1,chars:[{lowerCase:"'"}]},{width:.1,chars:[{lowerCase:":"}]},{width:.1,chars:[{lowerCase:";"}]},{width:.1,chars:[{lowerCase:"!"}]},{width:.1,chars:[{lowerCase:"?"}]},{width:.2,command:"backspace",chars:[{icon:"backspace"}]}],[{width:.2,command:"switch",chars:[{lowerCase:".?12"}]},{width:.1,chars:[{lowerCase:","}]},{width:.4,command:"space",chars:[{icon:"space"}]},{width:.1,chars:[{lowerCase:"."}]},{width:.2,command:"enter",chars:[{icon:"enter"}]}]]],el:[[[{width:.1,chars:[{lowerCase:";",upperCase:":"},{lowerCase:"q",upperCase:"Q"}]},{width:.1,chars:[{lowerCase:"\u03C2",upperCase:"\u03C2"},{lowerCase:"w",upperCase:"W"}]},{width:.1,chars:[{lowerCase:"\u03B5",upperCase:"\u0395"},{lowerCase:"e",upperCase:"E"}]},{width:.1,chars:[{lowerCase:"\u03C1",upperCase:"\u03A1"},{lowerCase:"r",upperCase:"R"}]},{width:.1,chars:[{lowerCase:"\u03C4",upperCase:"\u03A4"},{lowerCase:"t",upperCase:"T"}]},{width:.1,chars:[{lowerCase:"\u03C5",upperCase:"\u03A5"},{lowerCase:"y",upperCase:"Y"}]},{width:.1,chars:[{lowerCase:"\u03B8",upperCase:"\u0398"},{lowerCase:"u",upperCase:"U"}]},{width:.1,chars:[{lowerCase:"\u03B9",upperCase:"\u0399"},{lowerCase:"i",upperCase:"I"}]},{width:.1,chars:[{lowerCase:"\u03BF",upperCase:"\u039F"},{lowerCase:"o",upperCase:"O"}]},{width:.1,chars:[{lowerCase:"\u03C0",upperCase:"\u03A0"},{lowerCase:"p",upperCase:"P"}]}],[{width:.1,chars:[{lowerCase:"\u03B1",upperCase:"\u0391"},{lowerCase:"a",upperCase:"A"}]},{width:.1,chars:[{lowerCase:"\u03C3",upperCase:"\u03A3"},{lowerCase:"s",upperCase:"S"}]},{width:.1,chars:[{lowerCase:"\u03B4",upperCase:"\u0394"},{lowerCase:"d",upperCase:"D"}]},{width:.1,chars:[{lowerCase:"\u03C6",upperCase:"\u03A6"},{lowerCase:"f",upperCase:"F"}]},{width:.1,chars:[{lowerCase:"\u03B3",upperCase:"\u0393"},{lowerCase:"g",upperCase:"G"}]},{width:.1,chars:[{lowerCase:"\u03B7",upperCase:"\u0397"},{lowerCase:"h",upperCase:"H"}]},{width:.1,chars:[{lowerCase:"\u03BE",upperCase:"\u039E"},{lowerCase:"j",upperCase:"J"}]},{width:.1,chars:[{lowerCase:"\u03BA",upperCase:"\u039A"},{lowerCase:"k",upperCase:"K"}]},{width:.1,chars:[{lowerCase:"\u03BB",upperCase:"\u039B"},{lowerCase:"l",upperCase:"L"}]}],[{width:.15,command:"shift",chars:[{icon:"shift"}]},{width:.1,chars:[{lowerCase:"\u03B6",upperCase:"\u0396"},{lowerCase:"z",upperCase:"Z"}]},{width:.1,chars:[{lowerCase:"\u03C7",upperCase:"\u03A7"},{lowerCase:"x",upperCase:"X"}]},{width:.1,chars:[{lowerCase:"\u03C8",upperCase:"\u03A8"},{lowerCase:"c",upperCase:"C"}]},{width:.1,chars:[{lowerCase:"\u03C9",upperCase:"\u03A9"},{lowerCase:"v",upperCase:"V"}]},{width:.1,chars:[{lowerCase:"\u03B2",upperCase:"\u0392"},{lowerCase:"b",upperCase:"B"}]},{width:.1,chars:[{lowerCase:"\u03BD",upperCase:"\u039D"},{lowerCase:"n",upperCase:"N"}]},{width:.1,chars:[{lowerCase:"\u03BC",upperCase:"\u039C"},{lowerCase:"m",upperCase:"M"}]},{width:.15,command:"backspace",chars:[{icon:"backspace"}]}],[{width:.15,command:"switch-set",chars:[{lowerCase:"eng"}]},{width:.15,command:"switch",chars:[{lowerCase:".?12"}]},{width:.4,command:"space",chars:[{icon:"space"}]},{width:.1,chars:[{lowerCase:"?"}]},{width:.2,command:"enter",chars:[{icon:"enter"}]}]],[[{width:.1,chars:[{lowerCase:"1"}]},{width:.1,chars:[{lowerCase:"2"}]},{width:.1,chars:[{lowerCase:"3"}]},{width:.1,chars:[{lowerCase:"4"}]},{width:.1,chars:[{lowerCase:"5"}]},{width:.1,chars:[{lowerCase:"6"}]},{width:.1,chars:[{lowerCase:"7"}]},{width:.1,chars:[{lowerCase:"8"}]},{width:.1,chars:[{lowerCase:"9"}]},{width:.1,chars:[{lowerCase:"0"}]}],[{width:.1,chars:[{lowerCase:"@"}]},{width:.1,chars:[{lowerCase:"#"}]},{width:.1,chars:[{lowerCase:"|"}]},{width:.1,chars:[{lowerCase:"_"}]},{width:.1,chars:[{lowerCase:"&"}]},{width:.1,chars:[{lowerCase:"-"}]},{width:.1,chars:[{lowerCase:"+"}]},{width:.1,chars:[{lowerCase:"("}]},{width:.1,chars:[{lowerCase:")"}]},{width:.1,chars:[{lowerCase:"/"}]}],[{width:.1,chars:[{lowerCase:"="}]},{width:.1,chars:[{lowerCase:"*"}]},{width:.1,chars:[{lowerCase:'"'}]},{width:.1,chars:[{lowerCase:"'"}]},{width:.1,chars:[{lowerCase:":"}]},{width:.1,chars:[{lowerCase:";"}]},{width:.1,chars:[{lowerCase:"!"}]},{width:.1,chars:[{lowerCase:"?"}]},{width:.2,command:"backspace",chars:[{icon:"backspace"}]}],[{width:.2,command:"switch",chars:[{lowerCase:".?12"}]},{width:.1,chars:[{lowerCase:","}]},{width:.4,command:"space",chars:[{icon:"space"}]},{width:.1,chars:[{lowerCase:"."}]},{width:.2,command:"enter",chars:[{icon:"enter"}]}]]],nord:[[[{width:1/11,chars:[{lowerCase:"q",upperCase:"Q"}]},{width:1/11,chars:[{lowerCase:"w",upperCase:"W"}]},{width:1/11,chars:[{lowerCase:"e",upperCase:"E"}]},{width:1/11,chars:[{lowerCase:"r",upperCase:"R"}]},{width:1/11,chars:[{lowerCase:"t",upperCase:"T"}]},{width:1/11,chars:[{lowerCase:"y",upperCase:"Y"}]},{width:1/11,chars:[{lowerCase:"u",upperCase:"U"}]},{width:1/11,chars:[{lowerCase:"i",upperCase:"I"}]},{width:1/11,chars:[{lowerCase:"o",upperCase:"O"}]},{width:1/11,chars:[{lowerCase:"p",upperCase:"P"}]},{width:1/11,chars:[{lowerCase:"\xE5",upperCase:"\xC5"}]}],[{width:1/11,chars:[{lowerCase:"a",upperCase:"A"}]},{width:1/11,chars:[{lowerCase:"s",upperCase:"S"}]},{width:1/11,chars:[{lowerCase:"d",upperCase:"D"}]},{width:1/11,chars:[{lowerCase:"f",upperCase:"F"}]},{width:1/11,chars:[{lowerCase:"g",upperCase:"G"}]},{width:1/11,chars:[{lowerCase:"h",upperCase:"H"}]},{width:1/11,chars:[{lowerCase:"j",upperCase:"J"}]},{width:1/11,chars:[{lowerCase:"k",upperCase:"K"}]},{width:1/11,chars:[{lowerCase:"l",upperCase:"L"}]},{width:1/11,chars:[{lowerCase:"\xE6",upperCase:"\xC6"}]},{width:1/11,chars:[{lowerCase:"\xF8",upperCase:"\xD8"}]}],[{width:2/11,command:"shift",chars:[{icon:"shift"}]},{width:1/11,chars:[{lowerCase:"z",upperCase:"Z"}]},{width:1/11,chars:[{lowerCase:"x",upperCase:"X"}]},{width:1/11,chars:[{lowerCase:"c",upperCase:"C"}]},{width:1/11,chars:[{lowerCase:"v",upperCase:"V"}]},{width:1/11,chars:[{lowerCase:"b",upperCase:"B"}]},{width:1/11,chars:[{lowerCase:"n",upperCase:"N"}]},{width:1/11,chars:[{lowerCase:"m",upperCase:"M"}]},{width:2/11,command:"backspace",chars:[{icon:"backspace"}]}],[{width:.2,command:"switch",chars:[{lowerCase:".?12"}]},{width:.1,chars:[{lowerCase:","}]},{width:.4,command:"space",chars:[{icon:"space"}]},{width:.1,chars:[{lowerCase:"."}]},{width:.2,command:"enter",chars:[{icon:"enter"}]}]],[[{width:.1,chars:[{lowerCase:"1"}]},{width:.1,chars:[{lowerCase:"2"}]},{width:.1,chars:[{lowerCase:"3"}]},{width:.1,chars:[{lowerCase:"4"}]},{width:.1,chars:[{lowerCase:"5"}]},{width:.1,chars:[{lowerCase:"6"}]},{width:.1,chars:[{lowerCase:"7"}]},{width:.1,chars:[{lowerCase:"8"}]},{width:.1,chars:[{lowerCase:"9"}]},{width:.1,chars:[{lowerCase:"0"}]}],[{width:.1,chars:[{lowerCase:"@"}]},{width:.1,chars:[{lowerCase:"#"}]},{width:.1,chars:[{lowerCase:"|"}]},{width:.1,chars:[{lowerCase:"_"}]},{width:.1,chars:[{lowerCase:"&"}]},{width:.1,chars:[{lowerCase:"-"}]},{width:.1,chars:[{lowerCase:"+"}]},{width:.1,chars:[{lowerCase:"("}]},{width:.1,chars:[{lowerCase:")"}]},{width:.1,chars:[{lowerCase:"/"}]}],[{width:.1,chars:[{lowerCase:"="}]},{width:.1,chars:[{lowerCase:"*"}]},{width:.1,chars:[{lowerCase:'"'}]},{width:.1,chars:[{lowerCase:"'"}]},{width:.1,chars:[{lowerCase:":"}]},{width:.1,chars:[{lowerCase:";"}]},{width:.1,chars:[{lowerCase:"!"}]},{width:.1,chars:[{lowerCase:"?"}]},{width:.2,command:"backspace",chars:[{icon:"backspace"}]}],[{width:.2,command:"switch",chars:[{lowerCase:".?12"}]},{width:.1,chars:[{lowerCase:","}]},{width:.4,command:"space",chars:[{icon:"space"}]},{width:.1,chars:[{lowerCase:"."}]},{width:.2,command:"enter",chars:[{icon:"enter"}]}]]]},ab=new ke.TextureLoader,Gc=class extends Xs.withBase(ke.Object3D)(Bp,jc){constructor(e){e||(e={}),e.width||(e.width=1),e.height||(e.height=.4),e.margin||(e.margin=.003),e.padding||(e.padding=.01),super(e),this.currentPanel=0,this.isLowerCase=!0,this.charsetCount=1;let t;if(e.language||navigator.language)switch(e.language||navigator.language){case"fr":case"fr-CH":case"fr-CA":t=Ki.fr;break;case"ru":this.charsetCount=2,t=Ki.ru;break;case"de":case"de-DE":case"de-AT":case"de-LI":case"de-CH":t=Ki.de;break;case"es":case"es-419":case"es-AR":case"es-CL":case"es-CO":case"es-ES":case"es-CR":case"es-US":case"es-HN":case"es-MX":case"es-PE":case"es-UY":case"es-VE":t=Ki.es;break;case"el":this.charsetCount=2,t=Ki.el;break;case"nord":t=Ki.nord;break;default:t=Ki.eng;break}else t=Ki.eng;this.keys=[],this.panels=t.map(i=>{let r=e.height/i.length-e.margin*2,s=new Ur({width:e.width+e.padding*2,height:e.height+e.padding*2,offset:0,padding:e.padding,fontFamily:e.fontFamily,fontTexture:e.fontTexture,backgroundColor:e.backgroundColor,backgroundOpacity:e.backgroundOpacity});return s.charset=0,s.add(...i.map(o=>{let a=new Ur({width:e.width,height:r,margin:e.margin,contentDirection:"row",justifyContent:"center"});a.frame.visible=!1;let l=[];return o.forEach(c=>{let u=new Ur({width:e.width*c.width-e.margin*2,height:r,margin:e.margin,justifyContent:"center",offset:0}),p=c.chars[s.charset].lowerCase||c.chars[s.charset].icon||"undif";if(p==="enter"&&e.enterTexture||p==="shift"&&e.shiftTexture||p==="backspace"&&e.backspaceTexture){let f=(()=>{switch(p){case"backspace":return e.backspaceTexture;case"enter":return e.enterTexture;case"shift":return e.shiftTexture;default:console.warn("There is no icon image for this key")}})();ab.load(f,m=>{u.add(new pa({width:u.width*.65,height:u.height*.65,backgroundSize:"contain",backgroundTexture:m}))})}else u.add(new fa({content:p,offset:0}));u.type="Key",u.info=c,u.info.input=p,u.panel=s,l.push(u),this.keys.push(u)}),a.add(...l),a})),s}),this.add(this.panels[0]),this.set(e)}setNextPanel(){this.panels.forEach(e=>{this.remove(e)}),this.currentPanel=(this.currentPanel+1)%this.panels.length,this.add(this.panels[this.currentPanel]),this.update(!0,!0,!0)}setNextCharset(){this.panels[this.currentPanel].charset=(this.panels[this.currentPanel].charset+1)%this.charsetCount,this.keys.forEach(e=>{if(!this.panels[this.currentPanel].getObjectById(e.id))return;let i=e.info.chars[e.panel.charset]||e.info.chars[0],r=this.isLowerCase||!i.upperCase?i.lowerCase:i.upperCase;if(!e.childrenTexts.length)return;let s=e.childrenTexts[0];e.info.input=r,s.set({content:r}),s.update(!0,!0,!0)})}toggleCase(){this.isLowerCase=!this.isLowerCase,this.keys.forEach(e=>{let t=e.info.chars[e.panel.charset]||e.info.chars[0],i=this.isLowerCase||!t.upperCase?t.lowerCase:t.upperCase;if(!e.childrenTexts.length)return;let r=e.childrenTexts[0];e.info.input=i,r.set({content:i}),r.update(!0,!0,!0)})}parseParams(){}updateLayout(){}updateInner(){}},fy=()=>mi.update(),py={Block:Ur,Text:fa,InlineBlock:pa,Keyboard:Gc,FontLibrary:da,update:fy,TextAlign:$c,Whitespace:qc,JustifyContent:Yc,AlignItems:Xc,ContentDirection:Wc};typeof global<"u"&&(global.ThreeMeshUI=py);var lb=py,Fb=hn.g1,Bb=hn.gO,kb=hn.km,zb=hn.zV,Vb=hn.ol,Hb=hn.uM,Gb=hn.N1,Wb=hn.xv,Xb=hn.PH,Yb=hn.UH,Qc=hn.ZP,qb=hn.Vx;var eu=[0,1.7,0];var Gp=class{_container;constructor(e,t,i){this._container=new Qc.Block({width:e,height:t,textAlign:"center",justifyContent:"center",backgroundColor:new Oe(0),fontFamily:"./Roboto-msdf.json",fontTexture:"./Roboto-msdf.png",borderRadius:Math.min(e,t)/4});let r=new Qc.Text({content:i,fontSize:.03,fontColor:new Oe(16777215)});this._container.add(r)}get_container(){return this._container}},tu=class{m_nLastTime=0;m_scene;m_camera;m_renderer;m_renderTarget;m_xrSession;m_canvas;m_videoGroup;_funGetApp;_panel_ip_ui;getScene(){return this.m_scene}getCamera(){return this.m_camera}getRenderer(){return this.m_renderer}init(e,t){this._funGetApp=t,this.m_canvas=e,this.m_scene=new Mc,this.m_camera=new Yt(45,window.innerWidth/window.innerHeight,.001,1e3),this.m_scene.add(this.m_camera),this.m_camera.position.set(eu[0],eu[1],eu[2]),this.m_camera.lookAt(0,eu[1],-1),window.onresize=()=>{this.m_camera.aspect=window.innerWidth/window.innerHeight,this.m_camera.updateProjectionMatrix(),this.m_renderer.setSize(window.innerWidth,window.innerHeight)},window.addEventListener("error",r=>(r&&Fe.error(r.toString()),!0),!0),window.addEventListener("beforeunload",r=>{this.m_xrSession&&(Fe.serverLog("\u9000\u51FAXRSession"),this.m_xrSession.end())});let i=e.getContext("webgl2");this.m_renderer=new Sc({antialias:!0,canvas:this.m_canvas,context:i}),this.m_renderer.setPixelRatio(window.devicePixelRatio),this.m_renderer.setSize(window.innerWidth,window.innerHeight),this.m_renderer.shadowMap.enabled=!0,this.m_renderer.shadowMap.type=fp,this.m_renderer.xr.enabled=!0,this.m_renderer.setAnimationLoop(()=>{this.onRenderFrame()}),Fe.info("initScene end"),this.createScene(),Fe.info("createScene end"),this.initControllerModel(),Fe.info("initControllerModel end"),this.m_renderer.autoClearColor=!1}update_ip_ui(){if(this._panel_ip_ui&&this.m_camera.remove(this._panel_ip_ui.get_container()),!this._funGetApp){console.error("no app fun");return}let e=this._funGetApp();if(!e){console.error("no app");return}let t=e.getComVRInput(),i=t.get_pose_ip(),r=t.get_list_pose_ip(),s=`current:${i}
${r.join(`
`)}`;this._panel_ip_ui=new Gp(.5,.2,s),this._panel_ip_ui.get_container().position.set(0,.2,-.5),this.m_camera.add(this._panel_ip_ui.get_container())}startVR(){let e={optionalFeatures:["local-floor"]};Fe.info("requestSession start"),navigator.xr.requestSession("immersive-ar",e).then(async t=>{if(!t){Fe.error("no xrSession");return}this.m_xrSession=t,Fe.info("requestSession end"),await this.m_renderer.xr.setSession(t),Fe.info("setSession"),this.m_renderer.xr.updateCamera(this.m_camera),t.requestReferenceSpace("unbounded").then(i=>{let r=this._funGetApp();r.onXRRef&&r.onXRRef(t,i)})})}createScene(){let e=new Ic(16777215,1);e.castShadow=!1,this.m_scene.add(e);let t=new Pc(16777215,5);t.position.set(1,1,1),t.castShadow=!0,t.shadow.camera.top=10,t.shadow.camera.bottom=-10,t.shadow.camera.right=10,t.shadow.camera.left=-10,t.shadow.mapSize.set(4096,4096),this.m_scene.add(t);let i=new Yn(10,10),r=new oa({color:1118481}),s=new St(i,r);s.rotation.x=-Math.PI/2,s.receiveShadow=!0;let o=new oa({color:16777215}),a=new Pr(.2,.2,.2);new St(a,o).position.set(0,1.5,-2),this.m_videoGroup=new Xi,this.m_videoGroup.scale.set(1,1,1),this.m_camera.add(this.m_videoGroup);let c=this._funGetApp();if(c.getComStereo){let u=c.getComStereo();u&&u.setToScene(this.m_videoGroup,!0)}}initControllerModel(){}onRenderFrame(){let e=Fe.getNowSecond();if(this.m_nLastTime<=0){this.m_nLastTime=e;return}let t=e-this.m_nLastTime;if(this.m_nLastTime=e,!!this.m_renderer){if(Qc.update(),this.m_renderer.xr.isPresenting){let i=this.m_renderer.xr.getFrame(),r=this.m_renderer.xr.getReferenceSpace(),s=this._funGetApp();s.onXRFrame&&s.onXRFrame(i,r)}this.m_renderer.render(this.m_scene,this.m_camera)}}};var nu=class{_fun_get_app;_fun_change_ip=void 0;_n_ip_change_cd=0;constructor(e){this._fun_get_app=e}init(e){this._fun_change_ip=e}_on_change_ip(e){this._fun_change_ip&&this._fun_change_ip(e)}updateFrame(e){let t=this._fun_get_app().getComVRInput();this._n_ip_change_cd>0&&(this._n_ip_change_cd-=e);let i=t.getAxesValue("Left",2);i!==void 0&&Math.abs(i)>=.5&&this._n_ip_change_cd<=0&&(this._n_ip_change_cd=2,this._on_change_ip(i>0?1:-1))}};var iu=class n{static m_instance;static getApp(){return n.m_instance||(n.m_instance=new n),n.m_instance}m_comVRRender=new tu;m_comStereo=new Oc;m_comVRInput=new Dc;_com_vr_controller=new nu(()=>n.getApp());m_comRTCLeft=new ua;m_comRTCRight=new ua;m_strCameraType="None";async init(){this.showVRButton(!0);let e=await Fe.postRes("/config",{}),t=e.source_type,i=e.share_memory_name,r=e.camera_width,s=e.camera_height,o=e.vr_data_rate,a=e.ip;this.m_comVRInput.set_list_pose_ip([a]),this.m_comVRInput.change_list_pose_ip(1);let l=document.getElementById("video_rtc_left"),c=document.getElementById("video_rtc_right"),u=document.getElementById("btn_collect"),p=document.getElementById("canvas_render");this.m_comVRInput.registerBtnUp("left",4,()=>{this.m_comVRInput.change_list_pose_ip(1),this.m_comVRRender.update_ip_ui()}),this.m_comStereo.init(l,c,1),this.m_comVRRender.init(p,()=>n.getApp()),this.m_comVRRender.update_ip_ui(),u.onclick=()=>{this.showVRButton(!1),this.m_comVRRender.startVR()},this.m_comRTCLeft.init(t,r,s,{left:!0,shm:i},l),this.m_comRTCRight.init(t,r,s,{left:!1,shm:i},c);let f=1/30;setInterval(()=>{this.m_comVRInput.updateFrame(f)},1e3/30),Fe.info(`\u53D1\u9001VR\u6570\u636E\u9891\u7387:${o}`),setInterval(()=>{this.m_comVRInput.sendVRData()},1e3/o)}showVRButton(e){let t=document.getElementById("btn_collect");e?(t.style.opacity="1",t.style.pointerEvents="auto"):(t.style.opacity="0",t.style.pointerEvents="none")}getCameraType(){return this.m_strCameraType}getComVRRender(){return this.m_comVRRender}getComVRInput(){return this.m_comVRInput}getComStereo(){return this.m_comStereo}onXRFrame(e,t){this.m_comVRInput.onXRFrame(e,t)}};window.onload=()=>{iu.getApp().init()};function cb(){return En.default.createElement(En.default.Fragment,null,En.default.createElement("video",{id:"video_rtc_left",className:"video_rtc",autoPlay:!0,controls:!0}),En.default.createElement("video",{id:"video_rtc_right",className:"video_rtc",autoPlay:!0,controls:!0}),En.default.createElement("canvas",{id:"canvas_render"}),En.default.createElement("div",{id:"btn_collect"},En.default.createElement("div",{className:"txt_button"},"\u5F00\u59CB\u53D1\u9001\u6570\u636E")),En.default.createElement("div",{id:"txt_frame",className:"txt_info"}))}my.default.render(En.default.createElement(En.default.StrictMode,null,En.default.createElement(cb,null)),document.getElementById("root"));})();
/*! Bundled license information:

object-assign/index.js:
  (*
  object-assign
  (c) Sindre Sorhus
  @license MIT
  *)

react/cjs/react.production.min.js:
  (** @license React v17.0.2
   * react.production.min.js
   *
   * Copyright (c) Facebook, Inc. and its affiliates.
   *
   * This source code is licensed under the MIT license found in the
   * LICENSE file in the root directory of this source tree.
   *)

scheduler/cjs/scheduler.production.min.js:
  (** @license React v0.20.2
   * scheduler.production.min.js
   *
   * Copyright (c) Facebook, Inc. and its affiliates.
   *
   * This source code is licensed under the MIT license found in the
   * LICENSE file in the root directory of this source tree.
   *)

react-dom/cjs/react-dom.production.min.js:
  (** @license React v17.0.2
   * react-dom.production.min.js
   *
   * Copyright (c) Facebook, Inc. and its affiliates.
   *
   * This source code is licensed under the MIT license found in the
   * LICENSE file in the root directory of this source tree.
   *)

three/build/three.module.js:
  (**
   * @license
   * Copyright 2010-2024 Three.js Authors
   * SPDX-License-Identifier: MIT
   *)
*/
