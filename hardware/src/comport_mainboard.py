<!DOCTYPE html>
<html lang="en">
<head>
  <meta id="bb-bootstrap" data-current-user="{&quot;isKbdShortcutsEnabled&quot;: true, &quot;isSshEnabled&quot;: false, &quot;isAuthenticated&quot;: false}"
 />
  <script nonce="">

if (window.performance) {

  
  window.performance.okayToSendMetrics = !document.hidden && 'onvisibilitychange' in document;

  if (window.performance.okayToSendMetrics) {

    
    window.addEventListener('visibilitychange', function () {
      if (document.hidden) {
        window.performance.okayToSendMetrics = false;
      }
    });
  }
}
</script>
  
  
  <meta http-equiv="X-UA-Compatible" content="IE=edge" />
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <meta charset="utf-8">
  <title>
  MartinAppo / diploaf-2017 
  / source  / hardware_module / src / hardware_module / comport_mainboard.py
 &mdash; Bitbucket
</title>
  <script nonce="" type="text/javascript">(window.NREUM||(NREUM={})).loader_config={xpid:"VwMGVVZSGwIIUFBQDwU="};window.NREUM||(NREUM={}),__nr_require=function(t,e,n){function r(n){if(!e[n]){var o=e[n]={exports:{}};t[n][0].call(o.exports,function(e){var o=t[n][1][e];return r(o||e)},o,o.exports)}return e[n].exports}if("function"==typeof __nr_require)return __nr_require;for(var o=0;o<n.length;o++)r(n[o]);return r}({1:[function(t,e,n){function r(t){try{c.console&&console.log(t)}catch(e){}}var o,i=t("ee"),a=t(20),c={};try{o=localStorage.getItem("__nr_flags").split(","),console&&"function"==typeof console.log&&(c.console=!0,o.indexOf("dev")!==-1&&(c.dev=!0),o.indexOf("nr_dev")!==-1&&(c.nrDev=!0))}catch(s){}c.nrDev&&i.on("internal-error",function(t){r(t.stack)}),c.dev&&i.on("fn-err",function(t,e,n){r(n.stack)}),c.dev&&(r("NR AGENT IN DEVELOPMENT MODE"),r("flags: "+a(c,function(t,e){return t}).join(", ")))},{}],2:[function(t,e,n){function r(t,e,n,r,c){try{h?h-=1:o(c||new UncaughtException(t,e,n),!0)}catch(f){try{i("ierr",[f,s.now(),!0])}catch(d){}}return"function"==typeof u&&u.apply(this,a(arguments))}function UncaughtException(t,e,n){this.message=t||"Uncaught error with no additional information",this.sourceURL=e,this.line=n}function o(t,e){var n=e?null:s.now();i("err",[t,n])}var i=t("handle"),a=t(21),c=t("ee"),s=t("loader"),f=t("gos"),u=window.onerror,d=!1,p="nr@seenError",h=0;s.features.err=!0,t(1),window.onerror=r;try{throw new Error}catch(l){"stack"in l&&(t(13),t(12),"addEventListener"in window&&t(6),s.xhrWrappable&&t(14),d=!0)}c.on("fn-start",function(t,e,n){d&&(h+=1)}),c.on("fn-err",function(t,e,n){d&&!n[p]&&(f(n,p,function(){return!0}),this.thrown=!0,o(n))}),c.on("fn-end",function(){d&&!this.thrown&&h>0&&(h-=1)}),c.on("internal-error",function(t){i("ierr",[t,s.now(),!0])})},{}],3:[function(t,e,n){t("loader").features.ins=!0},{}],4:[function(t,e,n){function r(){M++,S=y.hash,this[u]=b.now()}function o(){M--,y.hash!==S&&i(0,!0);var t=b.now();this[l]=~~this[l]+t-this[u],this[d]=t}function i(t,e){E.emit("newURL",[""+y,e])}function a(t,e){t.on(e,function(){this[e]=b.now()})}var c="-start",s="-end",f="-body",u="fn"+c,d="fn"+s,p="cb"+c,h="cb"+s,l="jsTime",m="fetch",v="addEventListener",w=window,y=w.location,b=t("loader");if(w[v]&&b.xhrWrappable){var g=t(10),x=t(11),E=t(8),P=t(6),O=t(13),R=t(7),T=t(14),L=t(9),j=t("ee"),N=j.get("tracer");t(15),b.features.spa=!0;var S,M=0;j.on(u,r),j.on(p,r),j.on(d,o),j.on(h,o),j.buffer([u,d,"xhr-done","xhr-resolved"]),P.buffer([u]),O.buffer(["setTimeout"+s,"clearTimeout"+c,u]),T.buffer([u,"new-xhr","send-xhr"+c]),R.buffer([m+c,m+"-done",m+f+c,m+f+s]),E.buffer(["newURL"]),g.buffer([u]),x.buffer(["propagate",p,h,"executor-err","resolve"+c]),N.buffer([u,"no-"+u]),L.buffer(["new-jsonp","cb-start","jsonp-error","jsonp-end"]),a(T,"send-xhr"+c),a(j,"xhr-resolved"),a(j,"xhr-done"),a(R,m+c),a(R,m+"-done"),a(L,"new-jsonp"),a(L,"jsonp-end"),a(L,"cb-start"),E.on("pushState-end",i),E.on("replaceState-end",i),w[v]("hashchange",i,!0),w[v]("load",i,!0),w[v]("popstate",function(){i(0,M>1)},!0)}},{}],5:[function(t,e,n){function r(t){}if(window.performance&&window.performance.timing&&window.performance.getEntriesByType){var o=t("ee"),i=t("handle"),a=t(13),c=t(12),s="learResourceTimings",f="addEventListener",u="resourcetimingbufferfull",d="bstResource",p="resource",h="-start",l="-end",m="fn"+h,v="fn"+l,w="bstTimer",y="pushState",b=t("loader");b.features.stn=!0,t(8);var g=NREUM.o.EV;o.on(m,function(t,e){var n=t[0];n instanceof g&&(this.bstStart=b.now())}),o.on(v,function(t,e){var n=t[0];n instanceof g&&i("bst",[n,e,this.bstStart,b.now()])}),a.on(m,function(t,e,n){this.bstStart=b.now(),this.bstType=n}),a.on(v,function(t,e){i(w,[e,this.bstStart,b.now(),this.bstType])}),c.on(m,function(){this.bstStart=b.now()}),c.on(v,function(t,e){i(w,[e,this.bstStart,b.now(),"requestAnimationFrame"])}),o.on(y+h,function(t){this.time=b.now(),this.startPath=location.pathname+location.hash}),o.on(y+l,function(t){i("bstHist",[location.pathname+location.hash,this.startPath,this.time])}),f in window.performance&&(window.performance["c"+s]?window.performance[f](u,function(t){i(d,[window.performance.getEntriesByType(p)]),window.performance["c"+s]()},!1):window.performance[f]("webkit"+u,function(t){i(d,[window.performance.getEntriesByType(p)]),window.performance["webkitC"+s]()},!1)),document[f]("scroll",r,{passive:!0}),document[f]("keypress",r,!1),document[f]("click",r,!1)}},{}],6:[function(t,e,n){function r(t){for(var e=t;e&&!e.hasOwnProperty(u);)e=Object.getPrototypeOf(e);e&&o(e)}function o(t){c.inPlace(t,[u,d],"-",i)}function i(t,e){return t[1]}var a=t("ee").get("events"),c=t(23)(a,!0),s=t("gos"),f=XMLHttpRequest,u="addEventListener",d="removeEventListener";e.exports=a,"getPrototypeOf"in Object?(r(document),r(window),r(f.prototype)):f.prototype.hasOwnProperty(u)&&(o(window),o(f.prototype)),a.on(u+"-start",function(t,e){var n=t[1],r=s(n,"nr@wrapped",function(){function t(){if("function"==typeof n.handleEvent)return n.handleEvent.apply(n,arguments)}var e={object:t,"function":n}[typeof n];return e?c(e,"fn-",null,e.name||"anonymous"):n});this.wrapped=t[1]=r}),a.on(d+"-start",function(t){t[1]=this.wrapped||t[1]})},{}],7:[function(t,e,n){function r(t,e,n){var r=t[e];"function"==typeof r&&(t[e]=function(){var t=r.apply(this,arguments);return o.emit(n+"start",arguments,t),t.then(function(e){return o.emit(n+"end",[null,e],t),e},function(e){throw o.emit(n+"end",[e],t),e})})}var o=t("ee").get("fetch"),i=t(20);e.exports=o;var a=window,c="fetch-",s=c+"body-",f=["arrayBuffer","blob","json","text","formData"],u=a.Request,d=a.Response,p=a.fetch,h="prototype";u&&d&&p&&(i(f,function(t,e){r(u[h],e,s),r(d[h],e,s)}),r(a,"fetch",c),o.on(c+"end",function(t,e){var n=this;e?e.clone().arrayBuffer().then(function(t){n.rxSize=t.byteLength,o.emit(c+"done",[null,e],n)}):o.emit(c+"done",[t],n)}))},{}],8:[function(t,e,n){var r=t("ee").get("history"),o=t(23)(r);e.exports=r,o.inPlace(window.history,["pushState","replaceState"],"-")},{}],9:[function(t,e,n){function r(t){function e(){s.emit("jsonp-end",[],p),t.removeEventListener("load",e,!1),t.removeEventListener("error",n,!1)}function n(){s.emit("jsonp-error",[],p),s.emit("jsonp-end",[],p),t.removeEventListener("load",e,!1),t.removeEventListener("error",n,!1)}var r=t&&"string"==typeof t.nodeName&&"script"===t.nodeName.toLowerCase();if(r){var o="function"==typeof t.addEventListener;if(o){var a=i(t.src);if(a){var u=c(a),d="function"==typeof u.parent[u.key];if(d){var p={};f.inPlace(u.parent,[u.key],"cb-",p),t.addEventListener("load",e,!1),t.addEventListener("error",n,!1),s.emit("new-jsonp",[t.src],p)}}}}}function o(){return"addEventListener"in window}function i(t){var e=t.match(u);return e?e[1]:null}function a(t,e){var n=t.match(p),r=n[1],o=n[3];return o?a(o,e[r]):e[r]}function c(t){var e=t.match(d);return e&&e.length>=3?{key:e[2],parent:a(e[1],window)}:{key:t,parent:window}}var s=t("ee").get("jsonp"),f=t(23)(s);if(e.exports=s,o()){var u=/[?&](?:callback|cb)=([^&#]+)/,d=/(.*)\.([^.]+)/,p=/^(\w+)(\.|$)(.*)$/,h=["appendChild","insertBefore","replaceChild"];f.inPlace(HTMLElement.prototype,h,"dom-"),f.inPlace(HTMLHeadElement.prototype,h,"dom-"),f.inPlace(HTMLBodyElement.prototype,h,"dom-"),s.on("dom-start",function(t){r(t[0])})}},{}],10:[function(t,e,n){var r=t("ee").get("mutation"),o=t(23)(r),i=NREUM.o.MO;e.exports=r,i&&(window.MutationObserver=function(t){return this instanceof i?new i(o(t,"fn-")):i.apply(this,arguments)},MutationObserver.prototype=i.prototype)},{}],11:[function(t,e,n){function r(t){var e=a.context(),n=c(t,"executor-",e),r=new f(n);return a.context(r).getCtx=function(){return e},a.emit("new-promise",[r,e],e),r}function o(t,e){return e}var i=t(23),a=t("ee").get("promise"),c=i(a),s=t(20),f=NREUM.o.PR;e.exports=a,f&&(window.Promise=r,["all","race"].forEach(function(t){var e=f[t];f[t]=function(n){function r(t){return function(){a.emit("propagate",[null,!o],i),o=o||!t}}var o=!1;s(n,function(e,n){Promise.resolve(n).then(r("all"===t),r(!1))});var i=e.apply(f,arguments),c=f.resolve(i);return c}}),["resolve","reject"].forEach(function(t){var e=f[t];f[t]=function(t){var n=e.apply(f,arguments);return t!==n&&a.emit("propagate",[t,!0],n),n}}),f.prototype["catch"]=function(t){return this.then(null,t)},f.prototype=Object.create(f.prototype,{constructor:{value:r}}),s(Object.getOwnPropertyNames(f),function(t,e){try{r[e]=f[e]}catch(n){}}),a.on("executor-start",function(t){t[0]=c(t[0],"resolve-",this),t[1]=c(t[1],"resolve-",this)}),a.on("executor-err",function(t,e,n){t[1](n)}),c.inPlace(f.prototype,["then"],"then-",o),a.on("then-start",function(t,e){this.promise=e,t[0]=c(t[0],"cb-",this),t[1]=c(t[1],"cb-",this)}),a.on("then-end",function(t,e,n){this.nextPromise=n;var r=this.promise;a.emit("propagate",[r,!0],n)}),a.on("cb-end",function(t,e,n){a.emit("propagate",[n,!0],this.nextPromise)}),a.on("propagate",function(t,e,n){this.getCtx&&!e||(this.getCtx=function(){if(t instanceof Promise)var e=a.context(t);return e&&e.getCtx?e.getCtx():this})}),r.toString=function(){return""+f})},{}],12:[function(t,e,n){var r=t("ee").get("raf"),o=t(23)(r),i="equestAnimationFrame";e.exports=r,o.inPlace(window,["r"+i,"mozR"+i,"webkitR"+i,"msR"+i],"raf-"),r.on("raf-start",function(t){t[0]=o(t[0],"fn-")})},{}],13:[function(t,e,n){function r(t,e,n){t[0]=a(t[0],"fn-",null,n)}function o(t,e,n){this.method=n,this.timerDuration=isNaN(t[1])?0:+t[1],t[0]=a(t[0],"fn-",this,n)}var i=t("ee").get("timer"),a=t(23)(i),c="setTimeout",s="setInterval",f="clearTimeout",u="-start",d="-";e.exports=i,a.inPlace(window,[c,"setImmediate"],c+d),a.inPlace(window,[s],s+d),a.inPlace(window,[f,"clearImmediate"],f+d),i.on(s+u,r),i.on(c+u,o)},{}],14:[function(t,e,n){function r(t,e){d.inPlace(e,["onreadystatechange"],"fn-",c)}function o(){var t=this,e=u.context(t);t.readyState>3&&!e.resolved&&(e.resolved=!0,u.emit("xhr-resolved",[],t)),d.inPlace(t,y,"fn-",c)}function i(t){b.push(t),l&&(x?x.then(a):v?v(a):(E=-E,P.data=E))}function a(){for(var t=0;t<b.length;t++)r([],b[t]);b.length&&(b=[])}function c(t,e){return e}function s(t,e){for(var n in t)e[n]=t[n];return e}t(6);var f=t("ee"),u=f.get("xhr"),d=t(23)(u),p=NREUM.o,h=p.XHR,l=p.MO,m=p.PR,v=p.SI,w="readystatechange",y=["onload","onerror","onabort","onloadstart","onloadend","onprogress","ontimeout"],b=[];e.exports=u;var g=window.XMLHttpRequest=function(t){var e=new h(t);try{u.emit("new-xhr",[e],e),e.addEventListener(w,o,!1)}catch(n){try{u.emit("internal-error",[n])}catch(r){}}return e};if(s(h,g),g.prototype=h.prototype,d.inPlace(g.prototype,["open","send"],"-xhr-",c),u.on("send-xhr-start",function(t,e){r(t,e),i(e)}),u.on("open-xhr-start",r),l){var x=m&&m.resolve();if(!v&&!m){var E=1,P=document.createTextNode(E);new l(a).observe(P,{characterData:!0})}}else f.on("fn-end",function(t){t[0]&&t[0].type===w||a()})},{}],15:[function(t,e,n){function r(t){var e=this.params,n=this.metrics;if(!this.ended){this.ended=!0;for(var r=0;r<d;r++)t.removeEventListener(u[r],this.listener,!1);if(!e.aborted){if(n.duration=a.now()-this.startTime,4===t.readyState){e.status=t.status;var i=o(t,this.lastSize);if(i&&(n.rxSize=i),this.sameOrigin){var s=t.getResponseHeader("X-NewRelic-App-Data");s&&(e.cat=s.split(", ").pop())}}else e.status=0;n.cbTime=this.cbTime,f.emit("xhr-done",[t],t),c("xhr",[e,n,this.startTime])}}}function o(t,e){var n=t.responseType;if("json"===n&&null!==e)return e;var r="arraybuffer"===n||"blob"===n||"json"===n?t.response:t.responseText;return l(r)}function i(t,e){var n=s(e),r=t.params;r.host=n.hostname+":"+n.port,r.pathname=n.pathname,t.sameOrigin=n.sameOrigin}var a=t("loader");if(a.xhrWrappable){var c=t("handle"),s=t(16),f=t("ee"),u=["load","error","abort","timeout"],d=u.length,p=t("id"),h=t(19),l=t(18),m=window.XMLHttpRequest;a.features.xhr=!0,t(14),f.on("new-xhr",function(t){var e=this;e.totalCbs=0,e.called=0,e.cbTime=0,e.end=r,e.ended=!1,e.xhrGuids={},e.lastSize=null,h&&(h>34||h<10)||window.opera||t.addEventListener("progress",function(t){e.lastSize=t.loaded},!1)}),f.on("open-xhr-start",function(t){this.params={method:t[0]},i(this,t[1]),this.metrics={}}),f.on("open-xhr-end",function(t,e){"loader_config"in NREUM&&"xpid"in NREUM.loader_config&&this.sameOrigin&&e.setRequestHeader("X-NewRelic-ID",NREUM.loader_config.xpid)}),f.on("send-xhr-start",function(t,e){var n=this.metrics,r=t[0],o=this;if(n&&r){var i=l(r);i&&(n.txSize=i)}this.startTime=a.now(),this.listener=function(t){try{"abort"===t.type&&(o.params.aborted=!0),("load"!==t.type||o.called===o.totalCbs&&(o.onloadCalled||"function"!=typeof e.onload))&&o.end(e)}catch(n){try{f.emit("internal-error",[n])}catch(r){}}};for(var c=0;c<d;c++)e.addEventListener(u[c],this.listener,!1)}),f.on("xhr-cb-time",function(t,e,n){this.cbTime+=t,e?this.onloadCalled=!0:this.called+=1,this.called!==this.totalCbs||!this.onloadCalled&&"function"==typeof n.onload||this.end(n)}),f.on("xhr-load-added",function(t,e){var n=""+p(t)+!!e;this.xhrGuids&&!this.xhrGuids[n]&&(this.xhrGuids[n]=!0,this.totalCbs+=1)}),f.on("xhr-load-removed",function(t,e){var n=""+p(t)+!!e;this.xhrGuids&&this.xhrGuids[n]&&(delete this.xhrGuids[n],this.totalCbs-=1)}),f.on("addEventListener-end",function(t,e){e instanceof m&&"load"===t[0]&&f.emit("xhr-load-added",[t[1],t[2]],e)}),f.on("removeEventListener-end",function(t,e){e instanceof m&&"load"===t[0]&&f.emit("xhr-load-removed",[t[1],t[2]],e)}),f.on("fn-start",function(t,e,n){e instanceof m&&("onload"===n&&(this.onload=!0),("load"===(t[0]&&t[0].type)||this.onload)&&(this.xhrCbStart=a.now()))}),f.on("fn-end",function(t,e){this.xhrCbStart&&f.emit("xhr-cb-time",[a.now()-this.xhrCbStart,this.onload,e],e)})}},{}],16:[function(t,e,n){e.exports=function(t){var e=document.createElement("a"),n=window.location,r={};e.href=t,r.port=e.port;var o=e.href.split("://");!r.port&&o[1]&&(r.port=o[1].split("/")[0].split("@").pop().split(":")[1]),r.port&&"0"!==r.port||(r.port="https"===o[0]?"443":"80"),r.hostname=e.hostname||n.hostname,r.pathname=e.pathname,r.protocol=o[0],"/"!==r.pathname.charAt(0)&&(r.pathname="/"+r.pathname);var i=!e.protocol||":"===e.protocol||e.protocol===n.protocol,a=e.hostname===document.domain&&e.port===n.port;return r.sameOrigin=i&&(!e.hostname||a),r}},{}],17:[function(t,e,n){function r(){}function o(t,e,n){return function(){return i(t,[f.now()].concat(c(arguments)),e?null:this,n),e?void 0:this}}var i=t("handle"),a=t(20),c=t(21),s=t("ee").get("tracer"),f=t("loader"),u=NREUM;"undefined"==typeof window.newrelic&&(newrelic=u);var d=["setPageViewName","setCustomAttribute","setErrorHandler","finished","addToTrace","inlineHit","addRelease"],p="api-",h=p+"ixn-";a(d,function(t,e){u[e]=o(p+e,!0,"api")}),u.addPageAction=o(p+"addPageAction",!0),u.setCurrentRouteName=o(p+"routeName",!0),e.exports=newrelic,u.interaction=function(){return(new r).get()};var l=r.prototype={createTracer:function(t,e){var n={},r=this,o="function"==typeof e;return i(h+"tracer",[f.now(),t,n],r),function(){if(s.emit((o?"":"no-")+"fn-start",[f.now(),r,o],n),o)try{return e.apply(this,arguments)}catch(t){throw s.emit("fn-err",[arguments,this,t],n),t}finally{s.emit("fn-end",[f.now()],n)}}}};a("setName,setAttribute,save,ignore,onEnd,getContext,end,get".split(","),function(t,e){l[e]=o(h+e)}),newrelic.noticeError=function(t){"string"==typeof t&&(t=new Error(t)),i("err",[t,f.now()])}},{}],18:[function(t,e,n){e.exports=function(t){if("string"==typeof t&&t.length)return t.length;if("object"==typeof t){if("undefined"!=typeof ArrayBuffer&&t instanceof ArrayBuffer&&t.byteLength)return t.byteLength;if("undefined"!=typeof Blob&&t instanceof Blob&&t.size)return t.size;if(!("undefined"!=typeof FormData&&t instanceof FormData))try{return JSON.stringify(t).length}catch(e){return}}}},{}],19:[function(t,e,n){var r=0,o=navigator.userAgent.match(/Firefox[\/\s](\d+\.\d+)/);o&&(r=+o[1]),e.exports=r},{}],20:[function(t,e,n){function r(t,e){var n=[],r="",i=0;for(r in t)o.call(t,r)&&(n[i]=e(r,t[r]),i+=1);return n}var o=Object.prototype.hasOwnProperty;e.exports=r},{}],21:[function(t,e,n){function r(t,e,n){e||(e=0),"undefined"==typeof n&&(n=t?t.length:0);for(var r=-1,o=n-e||0,i=Array(o<0?0:o);++r<o;)i[r]=t[e+r];return i}e.exports=r},{}],22:[function(t,e,n){e.exports={exists:"undefined"!=typeof window.performance&&window.performance.timing&&"undefined"!=typeof window.performance.timing.navigationStart}},{}],23:[function(t,e,n){function r(t){return!(t&&t instanceof Function&&t.apply&&!t[a])}var o=t("ee"),i=t(21),a="nr@original",c=Object.prototype.hasOwnProperty,s=!1;e.exports=function(t,e){function n(t,e,n,o){function nrWrapper(){var r,a,c,s;try{a=this,r=i(arguments),c="function"==typeof n?n(r,a):n||{}}catch(f){p([f,"",[r,a,o],c])}u(e+"start",[r,a,o],c);try{return s=t.apply(a,r)}catch(d){throw u(e+"err",[r,a,d],c),d}finally{u(e+"end",[r,a,s],c)}}return r(t)?t:(e||(e=""),nrWrapper[a]=t,d(t,nrWrapper),nrWrapper)}function f(t,e,o,i){o||(o="");var a,c,s,f="-"===o.charAt(0);for(s=0;s<e.length;s++)c=e[s],a=t[c],r(a)||(t[c]=n(a,f?c+o:o,i,c))}function u(n,r,o){if(!s||e){var i=s;s=!0;try{t.emit(n,r,o,e)}catch(a){p([a,n,r,o])}s=i}}function d(t,e){if(Object.defineProperty&&Object.keys)try{var n=Object.keys(t);return n.forEach(function(n){Object.defineProperty(e,n,{get:function(){return t[n]},set:function(e){return t[n]=e,e}})}),e}catch(r){p([r])}for(var o in t)c.call(t,o)&&(e[o]=t[o]);return e}function p(e){try{t.emit("internal-error",e)}catch(n){}}return t||(t=o),n.inPlace=f,n.flag=a,n}},{}],ee:[function(t,e,n){function r(){}function o(t){function e(t){return t&&t instanceof r?t:t?s(t,c,i):i()}function n(n,r,o,i){if(!p.aborted||i){t&&t(n,r,o);for(var a=e(o),c=l(n),s=c.length,f=0;f<s;f++)c[f].apply(a,r);var d=u[y[n]];return d&&d.push([b,n,r,a]),a}}function h(t,e){w[t]=l(t).concat(e)}function l(t){return w[t]||[]}function m(t){return d[t]=d[t]||o(n)}function v(t,e){f(t,function(t,n){e=e||"feature",y[n]=e,e in u||(u[e]=[])})}var w={},y={},b={on:h,emit:n,get:m,listeners:l,context:e,buffer:v,abort:a,aborted:!1};return b}function i(){return new r}function a(){(u.api||u.feature)&&(p.aborted=!0,u=p.backlog={})}var c="nr@context",s=t("gos"),f=t(20),u={},d={},p=e.exports=o();p.backlog=u},{}],gos:[function(t,e,n){function r(t,e,n){if(o.call(t,e))return t[e];var r=n();if(Object.defineProperty&&Object.keys)try{return Object.defineProperty(t,e,{value:r,writable:!0,enumerable:!1}),r}catch(i){}return t[e]=r,r}var o=Object.prototype.hasOwnProperty;e.exports=r},{}],handle:[function(t,e,n){function r(t,e,n,r){o.buffer([t],r),o.emit(t,e,n)}var o=t("ee").get("handle");e.exports=r,r.ee=o},{}],id:[function(t,e,n){function r(t){var e=typeof t;return!t||"object"!==e&&"function"!==e?-1:t===window?0:a(t,i,function(){return o++})}var o=1,i="nr@id",a=t("gos");e.exports=r},{}],loader:[function(t,e,n){function r(){if(!x++){var t=g.info=NREUM.info,e=p.getElementsByTagName("script")[0];if(setTimeout(u.abort,3e4),!(t&&t.licenseKey&&t.applicationID&&e))return u.abort();f(y,function(e,n){t[e]||(t[e]=n)}),s("mark",["onload",a()+g.offset],null,"api");var n=p.createElement("script");n.src="https://"+t.agent,e.parentNode.insertBefore(n,e)}}function o(){"complete"===p.readyState&&i()}function i(){s("mark",["domContent",a()+g.offset],null,"api")}function a(){return E.exists&&performance.now?Math.round(performance.now()):(c=Math.max((new Date).getTime(),c))-g.offset}var c=(new Date).getTime(),s=t("handle"),f=t(20),u=t("ee"),d=window,p=d.document,h="addEventListener",l="attachEvent",m=d.XMLHttpRequest,v=m&&m.prototype;NREUM.o={ST:setTimeout,SI:d.setImmediate,CT:clearTimeout,XHR:m,REQ:d.Request,EV:d.Event,PR:d.Promise,MO:d.MutationObserver};var w=""+location,y={beacon:"bam.nr-data.net",errorBeacon:"bam.nr-data.net",agent:"js-agent.newrelic.com/nr-spa-1071.min.js"},b=m&&v&&v[h]&&!/CriOS/.test(navigator.userAgent),g=e.exports={offset:c,now:a,origin:w,features:{},xhrWrappable:b};t(17),p[h]?(p[h]("DOMContentLoaded",i,!1),d[h]("load",r,!1)):(p[l]("onreadystatechange",o),d[l]("onload",r)),s("mark",["firstbyte",c],null,"api");var x=0,E=t(22)},{}]},{},["loader",2,15,5,3,4]);</script>
  


<meta name="bb-env" content="production" />
<meta id="bb-canon-url" name="bb-canon-url" content="https://bitbucket.org">
<meta name="bb-api-canon-url" content="https://api.bitbucket.org">


<meta name="bb-commit-hash" content="4154430a88b8">
<meta name="bb-app-node" content="app-166">
<meta name="bb-view-name" content="bitbucket.apps.repo2.views.filebrowse">
<meta name="ignore-whitespace" content="False">
<meta name="tab-size" content="None">
<meta name="locale" content="en">
<meta name="application-name" content="Bitbucket">
<meta name="apple-mobile-web-app-title" content="Bitbucket">
<meta name="slack-app-id" content="A8W8QLZD1">


<meta name="theme-color" content="#0049B0">
<meta name="msapplication-TileColor" content="#0052CC">
<meta name="msapplication-TileImage" content="https://d301sr5gafysq2.cloudfront.net/4154430a88b8/img/logos/bitbucket/mstile-150x150.png">
<link rel="apple-touch-icon" sizes="180x180" type="image/png" href="https://d301sr5gafysq2.cloudfront.net/4154430a88b8/img/logos/bitbucket/apple-touch-icon.png">
<link rel="icon" sizes="192x192" type="image/png" href="https://d301sr5gafysq2.cloudfront.net/4154430a88b8/img/logos/bitbucket/android-chrome-192x192.png">

<link rel="icon" sizes="16x16 24x24 32x32 64x64" type="image/x-icon" href="/favicon.ico?v=2">
<link rel="mask-icon" href="https://d301sr5gafysq2.cloudfront.net/4154430a88b8/img/logos/bitbucket/safari-pinned-tab.svg" color="#0052CC">

<link rel="search" type="application/opensearchdescription+xml" href="/opensearch.xml" title="Bitbucket">

  <meta name="description" content="">
  
  
    
  



  <link rel="stylesheet" href="https://d301sr5gafysq2.cloudfront.net/4154430a88b8/css/entry/vendor.css" />
<link rel="stylesheet" href="https://d301sr5gafysq2.cloudfront.net/4154430a88b8/css/entry/app.css" />



  <link rel="stylesheet" href="https://d301sr5gafysq2.cloudfront.net/4154430a88b8/css/entry/adg3-skeleton-nav.css">
  <link rel="stylesheet" href="https://d301sr5gafysq2.cloudfront.net/4154430a88b8/css/entry/adg3.css">
  
  <script nonce="">
  window.__sentry__ = {"dsn": "https://ea49358f525d4019945839a3d7a8292a@sentry.io/159509", "release": "4154430a88b8 (production)", "tags": {"project": "bitbucket-core"}, "environment": "production"};
</script>
<script src="https://d301sr5gafysq2.cloudfront.net/4154430a88b8/dist/webpack/sentry.js" nonce=""></script>
  <script src="https://d301sr5gafysq2.cloudfront.net/4154430a88b8/dist/webpack/early.js" nonce=""></script>
  
  
  
    <link href="/MartinAppo/diploaf-2017/rss" rel="alternate nofollow" type="application/rss+xml" title="RSS feed for diploaf-2017" />

</head>
<body class="production adg3 "
    data-static-url="https://d301sr5gafysq2.cloudfront.net/4154430a88b8/"
data-base-url="https://bitbucket.org"
data-no-avatar-image="https://d301sr5gafysq2.cloudfront.net/4154430a88b8/img/default_avatar/user_blue.svg"
data-current-user="{&quot;isKbdShortcutsEnabled&quot;: true, &quot;isSshEnabled&quot;: false, &quot;isAuthenticated&quot;: false}"
data-atlassian-id="{&quot;loginStatusUrl&quot;: &quot;https://id.atlassian.com/profile/rest/profile&quot;}"
data-settings="{&quot;MENTIONS_MIN_QUERY_LENGTH&quot;: 3}"

data-current-repo="{&quot;scm&quot;: &quot;git&quot;, &quot;readOnly&quot;: false, &quot;mainbranch&quot;: {&quot;name&quot;: &quot;master&quot;}, &quot;uuid&quot;: &quot;c3518bc4-b447-4571-8f36-b7eb88577f94&quot;, &quot;language&quot;: &quot;&quot;, &quot;owner&quot;: {&quot;username&quot;: &quot;MartinAppo&quot;, &quot;uuid&quot;: &quot;5f380218-64ac-4571-a87e-d79700f27317&quot;, &quot;isTeam&quot;: false}, &quot;fullslug&quot;: &quot;MartinAppo/diploaf-2017&quot;, &quot;slug&quot;: &quot;diploaf-2017&quot;, &quot;id&quot;: 25842369, &quot;pygmentsLanguage&quot;: null}"
data-current-cset="0fa84b08f0b1b89afe857345207f6c0f02810df0"






data-browser-monitoring="true"
data-switch-create-pullrequest-commit-status="true"




>
<div
  id="page"
  data-mobile-nav
>
  
    <div id="adg3-navigation"
  
  
  
   data-mobile-nav
  >
  <nav class="skeleton-nav">
    <div class="skeleton-nav--left">
      <div class="skeleton-nav--left-top">
        <ul class="skeleton-nav--items">
          <li></li>
          <li></li>
          <li></li>
          <li class="skeleton--icon"></li>
          <li class="skeleton--icon-sub"></li>
          <li class="skeleton--icon-sub"></li>
          <li class="skeleton--icon-sub"></li>
          <li class="skeleton--icon-sub"></li>
          <li class="skeleton--icon-sub"></li>
          <li class="skeleton--icon-sub"></li>
        </ul>
      </div>
      <div class="skeleton-nav--left-bottom">
        <div class="skeleton-nav--left-bottom-wrapper">
          <div class="skeleton-nav--item-help"></div>
          <div class="skeleton-nav--item-profile"></div>
        </div>
      </div>
    </div>
    <div class="skeleton-nav--right">
      <ul class="skeleton-nav--items-wide">
        <li class="skeleton--icon-logo-container">
          <div class="skeleton--icon-container"></div>
          <div class="skeleton--icon-description"></div>
          <div class="skeleton--icon-logo"></div>
        </li>
        <li>
          <div class="skeleton--icon-small"></div>
          <div class="skeleton-nav--item-wide-content"></div>
        </li>
        <li>
          <div class="skeleton--icon-small"></div>
          <div class="skeleton-nav--item-wide-content"></div>
        </li>
        <li>
          <div class="skeleton--icon-small"></div>
          <div class="skeleton-nav--item-wide-content"></div>
        </li>
        <li>
          <div class="skeleton--icon-small"></div>
          <div class="skeleton-nav--item-wide-content"></div>
        </li>
        <li>
          <div class="skeleton--icon-small"></div>
          <div class="skeleton-nav--item-wide-content"></div>
        </li>
        <li>
          <div class="skeleton--icon-small"></div>
          <div class="skeleton-nav--item-wide-content"></div>
        </li>
      </ul>
    </div>
  </nav>
</div>

    <div id="wrapper">
      
  


      
  <div id="nps-survey-container"></div>

 

      
  

<div id="account-warning" data-module="header/account-warning"
  data-unconfirmed-addresses="false"
  data-no-addresses="false"
  
></div>



      
  
<header id="aui-message-bar">
  
</header>


      <div id="content" role="main">

        
          <header class="app-header">
            <div class="app-header--primary">
              
                <div class="app-header--context">
                  <div class="app-header--breadcrumbs">
                    
  <ol class="aui-nav aui-nav-breadcrumbs">
    <li>
  <a href="/MartinAppo/">Martin Appo</a>
</li>

<li>
  <a href="/MartinAppo/diploaf-2017">diploaf-2017</a>
</li>
    
  <li>
    <a href="/MartinAppo/diploaf-2017/src">
      Source
    </a>
  </li>

  </ol>

                  </div>
                  <h1 class="app-header--heading">
                    <span class="file-path">comport_mainboard.py</span>
                  </h1>
                </div>
              
            </div>

            <div class="app-header--secondary">
              
                
              
            </div>
          </header>
        

        
        
  <div class="aui-page-panel ">
    <div class="hidden">
  
  
  </div>
    <div class="aui-page-panel-inner">

      <div
        id="repo-content"
        class="aui-page-panel-content forks-enabled"
        data-module="repo/index"
        
      >
        
        
  <div id="source-container" class="maskable" data-module="repo/source/index">
    



<header id="source-path">
  
    <div class="labels labels-csv">
      <div class="aui-buttons">
        <button data-branches-tags-url="/api/1.0/repositories/MartinAppo/diploaf-2017/branches-tags"
                data-module="components/branch-dialog"
                
                class="aui-button branch-dialog-trigger" title="master">
          
            
              <span class="aui-icon aui-icon-small aui-iconfont-devtools-branch">Branch</span>
            
            <span class="name">master</span>
          
          <span class="aui-icon-dropdown"></span>
        </button>
        <button class="aui-button" id="checkout-branch-button"
                title="Check out this branch">
          <span class="aui-icon aui-icon-small aui-iconfont-devtools-clone">Check out branch</span>
          <span class="aui-icon-dropdown"></span>
        </button>
      </div>
      
    
    
  
    </div>
  
  
    <div class="secondary-actions">
      <div class="aui-buttons">
        
          <a href="/MartinAppo/diploaf-2017/src/0fa84b08f0b1/hardware_module/src/hardware_module/comport_mainboard.py?at=master"
            class="aui-button pjax-trigger source-toggle" aria-pressed="true">
            Source
          </a>
          <a href="/MartinAppo/diploaf-2017/diff/hardware_module/src/hardware_module/comport_mainboard.py?diff2=0fa84b08f0b1&at=master"
            class="aui-button pjax-trigger diff-toggle"
            title="Diff to previous change">
            Diff
          </a>
          <a href="/MartinAppo/diploaf-2017/history-node/0fa84b08f0b1/hardware_module/src/hardware_module/comport_mainboard.py?at=master"
            class="aui-button pjax-trigger history-toggle">
            History
          </a>
        
      </div>

      
      

    </div>
  
  <h1>
    
      
        <a href="/MartinAppo/diploaf-2017/src/0fa84b08f0b1?at=master"
          class="pjax-trigger root" title="MartinAppo/diploaf-2017 at 0fa84b08f0b1">diploaf-2017</a> /
      
      
        
          
            <a href="/MartinAppo/diploaf-2017/src/0fa84b08f0b1/hardware_module/?at=master"
              class="pjax-trigger directory-name">hardware_module</a> /
          
        
      
        
          
            <a href="/MartinAppo/diploaf-2017/src/0fa84b08f0b1/hardware_module/src/?at=master"
              class="pjax-trigger directory-name">src</a> /
          
        
      
        
          
            <a href="/MartinAppo/diploaf-2017/src/0fa84b08f0b1/hardware_module/src/hardware_module/?at=master"
              class="pjax-trigger directory-name">hardware_module</a> /
          
        
      
        
          
            <span class="file-name">comport_mainboard.py</span>
          
        
      
    
  </h1>
  
    
    
  
  <div class="clearfix"></div>
</header>


  
    
  

  <div id="editor-container" class="maskable"
       data-module="repo/source/editor"
       data-repo-id="25842369"
       data-owner="MartinAppo"
       data-slug="diploaf-2017"
       data-is-writer="false"
       data-has-push-access="true"
       data-hash="0fa84b08f0b1b89afe857345207f6c0f02810df0"
       data-branch="master"
       data-path="hardware_module/src/hardware_module/comport_mainboard.py"
       data-source-url="/api/internal/repositories/MartinAppo/diploaf-2017/src/0fa84b08f0b1b89afe857345207f6c0f02810df0/hardware_module/src/hardware_module/comport_mainboard.py">
    <div id="source-view" class="file-source-container" data-module="repo/source/view-file" data-is-lfs="false">
      <div class="toolbar">
        <div class="primary">
          <div class="aui-buttons">
            
              <button id="file-history-trigger" class="aui-button aui-button-light changeset-info"
                      data-changeset="0fa84b08f0b1b89afe857345207f6c0f02810df0"
                      data-path="hardware_module/src/hardware_module/comport_mainboard.py"
                      data-current="0fa84b08f0b1b89afe857345207f6c0f02810df0">
                 

  <div class="aui-avatar aui-avatar-xsmall">
    <div class="aui-avatar-inner">
      <img src="https://bitbucket.org/account/henryteigar/avatar/16/?ts=1535908022">
    </div>
  </div>
  <span class="changeset-hash">0fa84b0</span>
  <time datetime="2018-09-02T17:11:26+00:00" class="timestamp"></time>
  <span class="aui-icon-dropdown"></span>

              </button>
            
          </div>
          
          <a href="/MartinAppo/diploaf-2017/full-commit/0fa84b08f0b1/hardware_module/src/hardware_module/comport_mainboard.py" id="full-commit-link"
             title="View full commit 0fa84b0">Full commit</a>
        </div>
        <div class="secondary">
          
          <div class="aui-buttons">
            
              <a href="/MartinAppo/diploaf-2017/annotate/0fa84b08f0b1b89afe857345207f6c0f02810df0/hardware_module/src/hardware_module/comport_mainboard.py?at=master"
                 class="aui-button aui-button-light pjax-trigger blame-link">Annotate</a>
              
                
                <a data-embed-url="https://bitbucket.org/MartinAppo/diploaf-2017/src/0fa84b08f0b1b89afe857345207f6c0f02810df0/hardware_module/src/hardware_module/comport_mainboard.py?embed=t" class="aui-button aui-button-light js-embed-link">
                  Embed
                </a>
              
            
            <a href="/MartinAppo/diploaf-2017/raw/0fa84b08f0b1b89afe857345207f6c0f02810df0/hardware_module/src/hardware_module/comport_mainboard.py" class="aui-button aui-button-light raw-link">Raw</a>
          </div>
          
            <button id="file-edit-button" class="edit-button aui-button aui-button-light" disabled="disabled" aria-disabled="true">
              Edit
              <span class="edit-button-overlay" title="Log in to edit this file"></span>
            </button>
          
        </div>

        <div id="fileview-dropdown"
            class="aui-dropdown2 aui-style-default"
            data-fileview-container="#fileview-container"
            
            
            data-fileview-button="#fileview-trigger"
            data-module="connect/fileview">
          <div class="aui-dropdown2-section">
            <ul>
              <li>
                <a class="aui-dropdown2-radio aui-dropdown2-checked"
                    data-fileview-id="-1"
                    data-fileview-loaded="true"
                    data-fileview-target="#fileview-original"
                    data-fileview-connection-key=""
                    data-fileview-module-key="file-view-default">
                  Default File Viewer
                </a>
              </li>
              
            </ul>
          </div>
        </div>

        <div class="clearfix"></div>
      </div>
      <div id="fileview-container">
        <div id="fileview-original"
            class="fileview"
            data-module="repo/source/highlight-lines"
            data-fileview-loaded="true">
          


  
    <div class="file-source">
      <table class="codehilite highlighttable"><tr><td class="linenos"><div class="linenodiv"><pre><a href="#comport_mainboard.py-1"> 1</a>
<a href="#comport_mainboard.py-2"> 2</a>
<a href="#comport_mainboard.py-3"> 3</a>
<a href="#comport_mainboard.py-4"> 4</a>
<a href="#comport_mainboard.py-5"> 5</a>
<a href="#comport_mainboard.py-6"> 6</a>
<a href="#comport_mainboard.py-7"> 7</a>
<a href="#comport_mainboard.py-8"> 8</a>
<a href="#comport_mainboard.py-9"> 9</a>
<a href="#comport_mainboard.py-10">10</a>
<a href="#comport_mainboard.py-11">11</a>
<a href="#comport_mainboard.py-12">12</a>
<a href="#comport_mainboard.py-13">13</a>
<a href="#comport_mainboard.py-14">14</a>
<a href="#comport_mainboard.py-15">15</a>
<a href="#comport_mainboard.py-16">16</a>
<a href="#comport_mainboard.py-17">17</a>
<a href="#comport_mainboard.py-18">18</a>
<a href="#comport_mainboard.py-19">19</a>
<a href="#comport_mainboard.py-20">20</a>
<a href="#comport_mainboard.py-21">21</a>
<a href="#comport_mainboard.py-22">22</a>
<a href="#comport_mainboard.py-23">23</a>
<a href="#comport_mainboard.py-24">24</a>
<a href="#comport_mainboard.py-25">25</a>
<a href="#comport_mainboard.py-26">26</a>
<a href="#comport_mainboard.py-27">27</a>
<a href="#comport_mainboard.py-28">28</a>
<a href="#comport_mainboard.py-29">29</a>
<a href="#comport_mainboard.py-30">30</a>
<a href="#comport_mainboard.py-31">31</a>
<a href="#comport_mainboard.py-32">32</a>
<a href="#comport_mainboard.py-33">33</a>
<a href="#comport_mainboard.py-34">34</a>
<a href="#comport_mainboard.py-35">35</a>
<a href="#comport_mainboard.py-36">36</a>
<a href="#comport_mainboard.py-37">37</a>
<a href="#comport_mainboard.py-38">38</a>
<a href="#comport_mainboard.py-39">39</a>
<a href="#comport_mainboard.py-40">40</a>
<a href="#comport_mainboard.py-41">41</a>
<a href="#comport_mainboard.py-42">42</a>
<a href="#comport_mainboard.py-43">43</a>
<a href="#comport_mainboard.py-44">44</a>
<a href="#comport_mainboard.py-45">45</a>
<a href="#comport_mainboard.py-46">46</a>
<a href="#comport_mainboard.py-47">47</a>
<a href="#comport_mainboard.py-48">48</a>
<a href="#comport_mainboard.py-49">49</a>
<a href="#comport_mainboard.py-50">50</a>
<a href="#comport_mainboard.py-51">51</a>
<a href="#comport_mainboard.py-52">52</a>
<a href="#comport_mainboard.py-53">53</a>
<a href="#comport_mainboard.py-54">54</a>
<a href="#comport_mainboard.py-55">55</a>
<a href="#comport_mainboard.py-56">56</a>
<a href="#comport_mainboard.py-57">57</a>
<a href="#comport_mainboard.py-58">58</a>
<a href="#comport_mainboard.py-59">59</a>
<a href="#comport_mainboard.py-60">60</a>
<a href="#comport_mainboard.py-61">61</a>
<a href="#comport_mainboard.py-62">62</a>
<a href="#comport_mainboard.py-63">63</a>
<a href="#comport_mainboard.py-64">64</a>
<a href="#comport_mainboard.py-65">65</a>
<a href="#comport_mainboard.py-66">66</a>
<a href="#comport_mainboard.py-67">67</a></pre></div></td><td class="code"><div class="codehilite highlight"><pre><span></span><a name="comport_mainboard.py-1"></a><span class="kn">import</span> <span class="nn">serial</span>
<a name="comport_mainboard.py-2"></a><span class="kn">import</span> <span class="nn">threading</span>
<a name="comport_mainboard.py-3"></a><span class="kn">import</span> <span class="nn">time</span>
<a name="comport_mainboard.py-4"></a><span class="kn">import</span> <span class="nn">subprocess</span>
<a name="comport_mainboard.py-5"></a><span class="kn">import</span> <span class="nn">rospy</span>
<a name="comport_mainboard.py-6"></a>
<a name="comport_mainboard.py-7"></a>
<a name="comport_mainboard.py-8"></a><span class="k">class</span> <span class="nc">ComportMainboard</span><span class="p">(</span><span class="n">threading</span><span class="o">.</span><span class="n">Thread</span><span class="p">):</span>
<a name="comport_mainboard.py-9"></a>    <span class="n">connection</span> <span class="o">=</span> <span class="bp">None</span>
<a name="comport_mainboard.py-10"></a>    <span class="n">connection_opened</span> <span class="o">=</span> <span class="bp">False</span>
<a name="comport_mainboard.py-11"></a>
<a name="comport_mainboard.py-12"></a>    <span class="k">def</span> <span class="fm">__init__</span><span class="p">(</span><span class="bp">self</span><span class="p">):</span>
<a name="comport_mainboard.py-13"></a>        <span class="n">threading</span><span class="o">.</span><span class="n">Thread</span><span class="o">.</span><span class="fm">__init__</span><span class="p">(</span><span class="bp">self</span><span class="p">)</span>
<a name="comport_mainboard.py-14"></a>
<a name="comport_mainboard.py-15"></a>    <span class="k">def</span> <span class="nf">open</span><span class="p">(</span><span class="bp">self</span><span class="p">):</span>
<a name="comport_mainboard.py-16"></a>        <span class="k">try</span><span class="p">:</span>
<a name="comport_mainboard.py-17"></a>            <span class="n">ports</span> <span class="o">=</span> <span class="n">subprocess</span><span class="o">.</span><span class="n">check_output</span><span class="p">(</span><span class="s1">&#39;ls /dev/ttyACM0&#39;</span><span class="p">,</span> <span class="n">shell</span><span class="o">=</span><span class="bp">True</span><span class="p">)</span><span class="o">.</span><span class="n">split</span><span class="p">(</span><span class="s1">&#39;</span><span class="se">\n</span><span class="s1">&#39;</span><span class="p">)[:</span><span class="o">-</span><span class="mi">1</span><span class="p">]</span>
<a name="comport_mainboard.py-18"></a>        <span class="k">except</span><span class="p">:</span>
<a name="comport_mainboard.py-19"></a>            <span class="k">print</span><span class="p">(</span><span class="s1">&#39;mainboard: /dev/ttyACM empty&#39;</span><span class="p">)</span>
<a name="comport_mainboard.py-20"></a>            <span class="k">return</span> <span class="bp">False</span>
<a name="comport_mainboard.py-21"></a>        <span class="bp">self</span><span class="o">.</span><span class="n">connection_opened</span> <span class="o">=</span> <span class="bp">False</span>
<a name="comport_mainboard.py-22"></a>        <span class="k">for</span> <span class="n">port</span> <span class="ow">in</span> <span class="n">ports</span><span class="p">:</span>  <span class="c1"># analyze serial ports</span>
<a name="comport_mainboard.py-23"></a>            <span class="k">try</span><span class="p">:</span>
<a name="comport_mainboard.py-24"></a>                <span class="k">while</span> <span class="ow">not</span> <span class="bp">self</span><span class="o">.</span><span class="n">connection_opened</span> <span class="ow">and</span> <span class="ow">not</span> <span class="n">rospy</span><span class="o">.</span><span class="n">is_shutdown</span><span class="p">():</span>
<a name="comport_mainboard.py-25"></a>                    <span class="bp">self</span><span class="o">.</span><span class="n">connection</span> <span class="o">=</span> <span class="n">serial</span><span class="o">.</span><span class="n">Serial</span><span class="p">(</span><span class="n">port</span><span class="p">,</span> <span class="n">baudrate</span><span class="o">=</span><span class="mi">115200</span><span class="p">,</span> <span class="n">timeout</span><span class="o">=</span><span class="mf">0.8</span><span class="p">,</span> <span class="n">dsrdtr</span><span class="o">=</span><span class="bp">True</span><span class="p">)</span>
<a name="comport_mainboard.py-26"></a>                    <span class="bp">self</span><span class="o">.</span><span class="n">connection_opened</span> <span class="o">=</span> <span class="bp">self</span><span class="o">.</span><span class="n">connection</span><span class="o">.</span><span class="n">isOpen</span><span class="p">()</span>
<a name="comport_mainboard.py-27"></a>                    <span class="n">time</span><span class="o">.</span><span class="n">sleep</span><span class="p">(</span><span class="mf">0.5</span><span class="p">)</span>
<a name="comport_mainboard.py-28"></a>                <span class="bp">self</span><span class="o">.</span><span class="n">connection</span><span class="o">.</span><span class="n">flush</span><span class="p">()</span>
<a name="comport_mainboard.py-29"></a>                <span class="k">print</span> <span class="s2">&quot;mainboard: Port opened successfully&quot;</span>
<a name="comport_mainboard.py-30"></a>            <span class="k">except</span> <span class="ne">Exception</span> <span class="k">as</span> <span class="n">e</span><span class="p">:</span>
<a name="comport_mainboard.py-31"></a>                <span class="k">print</span><span class="p">(</span><span class="n">e</span><span class="p">)</span>
<a name="comport_mainboard.py-32"></a>                <span class="k">continue</span>
<a name="comport_mainboard.py-33"></a>
<a name="comport_mainboard.py-34"></a>        <span class="k">return</span> <span class="bp">self</span><span class="o">.</span><span class="n">connection_opened</span>
<a name="comport_mainboard.py-35"></a>
<a name="comport_mainboard.py-36"></a>    <span class="k">def</span> <span class="nf">write</span><span class="p">(</span><span class="bp">self</span><span class="p">,</span> <span class="n">comm</span><span class="p">):</span>
<a name="comport_mainboard.py-37"></a>        <span class="k">if</span> <span class="bp">self</span><span class="o">.</span><span class="n">connection</span> <span class="ow">is</span> <span class="ow">not</span> <span class="bp">None</span><span class="p">:</span>
<a name="comport_mainboard.py-38"></a>            <span class="k">try</span><span class="p">:</span>
<a name="comport_mainboard.py-39"></a>                <span class="bp">self</span><span class="o">.</span><span class="n">connection</span><span class="o">.</span><span class="n">write</span><span class="p">(</span><span class="n">comm</span> <span class="o">+</span> <span class="s1">&#39;</span><span class="se">\n</span><span class="s1">&#39;</span><span class="p">)</span>
<a name="comport_mainboard.py-40"></a>            <span class="k">except</span><span class="p">:</span>
<a name="comport_mainboard.py-41"></a>                <span class="k">print</span><span class="p">(</span><span class="s1">&#39;mainboard: err write &#39;</span> <span class="o">+</span> <span class="n">comm</span><span class="p">)</span>
<a name="comport_mainboard.py-42"></a>
<a name="comport_mainboard.py-43"></a>    <span class="k">def</span> <span class="nf">servo</span><span class="p">(</span><span class="bp">self</span><span class="p">,</span> <span class="n">value</span><span class="p">):</span>
<a name="comport_mainboard.py-44"></a>        <span class="n">msg</span> <span class="o">=</span> <span class="s2">&quot;v{}&quot;</span><span class="o">.</span><span class="n">format</span><span class="p">(</span><span class="n">value</span><span class="p">)</span>
<a name="comport_mainboard.py-45"></a>        <span class="k">if</span> <span class="bp">self</span><span class="o">.</span><span class="n">connection_opened</span><span class="p">:</span>
<a name="comport_mainboard.py-46"></a>            <span class="bp">self</span><span class="o">.</span><span class="n">write</span><span class="p">(</span><span class="n">msg</span><span class="p">)</span>
<a name="comport_mainboard.py-47"></a>
<a name="comport_mainboard.py-48"></a>    <span class="k">def</span> <span class="nf">launch_motor</span><span class="p">(</span><span class="bp">self</span><span class="p">,</span> <span class="n">value</span><span class="p">):</span>
<a name="comport_mainboard.py-49"></a>        <span class="k">if</span> <span class="bp">self</span><span class="o">.</span><span class="n">connection_opened</span><span class="p">:</span>
<a name="comport_mainboard.py-50"></a>            <span class="bp">self</span><span class="o">.</span><span class="n">write</span><span class="p">(</span><span class="s2">&quot;d{}&quot;</span><span class="o">.</span><span class="n">format</span><span class="p">(</span><span class="n">value</span><span class="p">))</span>
<a name="comport_mainboard.py-51"></a>
<a name="comport_mainboard.py-52"></a>    <span class="k">def</span> <span class="nf">close</span><span class="p">(</span><span class="bp">self</span><span class="p">):</span>
<a name="comport_mainboard.py-53"></a>        <span class="k">if</span> <span class="bp">self</span><span class="o">.</span><span class="n">connection</span> <span class="ow">is</span> <span class="ow">not</span> <span class="bp">None</span> <span class="ow">and</span> <span class="bp">self</span><span class="o">.</span><span class="n">connection</span><span class="o">.</span><span class="n">isOpen</span><span class="p">():</span>  <span class="c1"># close coil</span>
<a name="comport_mainboard.py-54"></a>            <span class="k">try</span><span class="p">:</span>
<a name="comport_mainboard.py-55"></a>                <span class="bp">self</span><span class="o">.</span><span class="n">connection</span><span class="o">.</span><span class="n">close</span><span class="p">()</span>
<a name="comport_mainboard.py-56"></a>                <span class="k">print</span><span class="p">(</span><span class="s1">&#39;mainboard: connection closed&#39;</span><span class="p">)</span>
<a name="comport_mainboard.py-57"></a>            <span class="k">except</span><span class="p">:</span>
<a name="comport_mainboard.py-58"></a>                <span class="k">print</span><span class="p">(</span><span class="s1">&#39;mainboard: err connection close&#39;</span><span class="p">)</span>
<a name="comport_mainboard.py-59"></a>            <span class="bp">self</span><span class="o">.</span><span class="n">connection</span> <span class="o">=</span> <span class="bp">None</span>
<a name="comport_mainboard.py-60"></a>
<a name="comport_mainboard.py-61"></a>    <span class="k">def</span> <span class="nf">run</span><span class="p">(</span><span class="bp">self</span><span class="p">):</span>
<a name="comport_mainboard.py-62"></a>        <span class="k">if</span> <span class="bp">self</span><span class="o">.</span><span class="n">open</span><span class="p">():</span>  <span class="c1"># open serial connections</span>
<a name="comport_mainboard.py-63"></a>            <span class="k">print</span><span class="p">(</span><span class="s1">&#39;mainboard: opened&#39;</span><span class="p">)</span>
<a name="comport_mainboard.py-64"></a>        <span class="k">else</span><span class="p">:</span>
<a name="comport_mainboard.py-65"></a>            <span class="k">print</span><span class="p">(</span><span class="s1">&#39;mainboard: opening failed&#39;</span><span class="p">)</span>
<a name="comport_mainboard.py-66"></a>            <span class="bp">self</span><span class="o">.</span><span class="n">close</span><span class="p">()</span>
<a name="comport_mainboard.py-67"></a>            <span class="k">return</span>
</pre></div>
</td></tr></table>
    </div>
  


        </div>
        
      </div>
    </div>
  </div>
  
  <div data-module="source/set-changeset" data-hash="0fa84b08f0b1b89afe857345207f6c0f02810df0"></div>



  
    
    
    
  
  

  </div>

        
        
        
      </div>
    </div>
  </div>

      </div>
    </div>
  
  <div id="adg3-flag-root"></div>
</div>

<div id="adg3-dialog"></div>


  

<div data-module="components/mentions/index">
  
    
    
  
  
    
    
  
  
    
    
  
</div>
<div data-module="components/typeahead/emoji/index">
  
    
    
  
</div>

<div data-module="components/repo-typeahead/index">
  
    
    
  
</div>

    
    
  

    
    
  

    
    
  

    
    
  


  


    
    
  

    
    
  



  
  
  <aui-inline-dialog
    id="help-menu-dialog"
    data-aui-alignment="bottom right"

    
    data-aui-alignment-static="true"
    data-module="header/help-menu"
    responds-to="toggle"
    aria-hidden="true">

  <div id="help-menu-section">
    <h1 class="help-menu-heading">Help</h1>

    <form id="help-menu-search-form" class="aui" target="_blank" method="get"
        action="https://support.atlassian.com/customer/search">
      <span id="help-menu-search-icon" class="aui-icon aui-icon-large aui-iconfont-search"></span>
      <input id="help-menu-search-form-input" name="q" class="text" type="text" placeholder="Ask a question">
    </form>

    <ul id="help-menu-links">
      <li>
        <a class="support-ga" data-support-gaq-page="DocumentationHome"
            href="https://confluence.atlassian.com/x/bgozDQ" target="_blank">
          Online help
        </a>
      </li>
      <li>
        <a class="support-ga" data-support-gaq-page="GitTutorials"
            href="https://www.atlassian.com/git?utm_source=bitbucket&amp;utm_medium=link&amp;utm_campaign=help_dropdown&amp;utm_content=learn_git"
            target="_blank">
          Learn Git
        </a>
      </li>
      <li>
        <a id="keyboard-shortcuts-link"
           href="#">Keyboard shortcuts</a>
      </li>
      <li>
        <a class="support-ga" data-support-gaq-page="DocumentationTutorials"
            href="https://confluence.atlassian.com/x/Q4sFLQ" target="_blank">
          Bitbucket tutorials
        </a>
      </li>
      <li>
        <a class="support-ga" data-support-gaq-page="SiteStatus"
            href="https://status.bitbucket.org/" target="_blank">
          Site status
        </a>
      </li>
      <li>
        <a class="support-ga" data-support-gaq-page="Home"
            href="https://support.atlassian.com/bitbucket-cloud/" target="_blank">
          Support
        </a>
      </li>
    </ul>
  </div>
</aui-inline-dialog>
  






  

  <div class="_mustache-templates">
    
      <script id="branch-checkout-template" type="text/html">
        

<div id="checkout-branch-contents">
  <div class="command-line">
    <p>
      Check out this branch on your local machine to begin working on it.
    </p>
    <input type="text" class="checkout-command" readonly="readonly"
        
           value="git fetch && git checkout [[branchName]]"
        
        >
  </div>
  
    <div class="sourcetree-callout clone-in-sourcetree"
  data-module="components/clone/clone-in-sourcetree"
  data-https-url="https://bitbucket.org/MartinAppo/diploaf-2017.git"
  data-ssh-url="git@bitbucket.org:MartinAppo/diploaf-2017.git">

  <div>
    <button class="aui-button aui-button-primary">
      
        Check out in Sourcetree
      
    </button>
  </div>

  <p class="windows-text">
    
      <a href="http://www.sourcetreeapp.com/?utm_source=internal&amp;utm_medium=link&amp;utm_campaign=clone_repo_win" target="_blank">Atlassian Sourcetree</a>
      is a free Git and Mercurial client for Windows.
    
  </p>
  <p class="mac-text">
    
      <a href="http://www.sourcetreeapp.com/?utm_source=internal&amp;utm_medium=link&amp;utm_campaign=clone_repo_mac" target="_blank">Atlassian Sourcetree</a>
      is a free Git and Mercurial client for Mac.
    
  </p>
</div>
  
</div>

      </script>
    
      <script id="branch-dialog-template" type="text/html">
        

<div class="tabbed-filter-widget branch-dialog">
  <div class="tabbed-filter">
    <input placeholder="Filter branches" class="filter-box" autosave="branch-dropdown-25842369" type="text">
    [[^ignoreTags]]
      <div class="aui-tabs horizontal-tabs aui-tabs-disabled filter-tabs">
        <ul class="tabs-menu">
          <li class="menu-item active-tab"><a href="#branches">Branches</a></li>
          <li class="menu-item"><a href="#tags">Tags</a></li>
        </ul>
      </div>
    [[/ignoreTags]]
  </div>
  
    <div class="tab-pane active-pane" id="branches" data-filter-placeholder="Filter branches">
      <ol class="filter-list">
        <li class="empty-msg">No matching branches</li>
        [[#branches]]
          
            [[#hasMultipleHeads]]
              [[#heads]]
                <li class="comprev filter-item">
                  <a class="pjax-trigger filter-item-link" href="/MartinAppo/diploaf-2017/src/[[changeset]]/hardware_module/src/hardware_module/comport_mainboard.py?at=[[safeName]]"
                     title="[[name]]">
                    [[name]] ([[shortChangeset]])
                  </a>
                </li>
              [[/heads]]
            [[/hasMultipleHeads]]
            [[^hasMultipleHeads]]
              <li class="comprev filter-item">
                <a class="pjax-trigger filter-item-link" href="/MartinAppo/diploaf-2017/src/[[changeset]]/hardware_module/src/hardware_module/comport_mainboard.py?at=[[safeName]]" title="[[name]]">
                  [[name]]
                </a>
              </li>
            [[/hasMultipleHeads]]
          
        [[/branches]]
      </ol>
    </div>
    <div class="tab-pane" id="tags" data-filter-placeholder="Filter tags">
      <ol class="filter-list">
        <li class="empty-msg">No matching tags</li>
        [[#tags]]
          <li class="comprev filter-item">
            <a class="pjax-trigger filter-item-link" href="/MartinAppo/diploaf-2017/src/[[changeset]]/hardware_module/src/hardware_module/comport_mainboard.py?at=[[safeName]]" title="[[name]]">
              [[name]]
            </a>
          </li>
        [[/tags]]
      </ol>
    </div>
  
</div>

      </script>
    
      <script id="mention-result" type="text/html">
        
<span class="mention-result">
  <span class="aui-avatar aui-avatar-small mention-result--avatar">
    <span class="aui-avatar-inner">
      <img src="[[avatar_url]]">
    </span>
  </span>
  [[#display_name]]
    <span class="display-name mention-result--display-name">[[&display_name]]</span> <small class="username mention-result--username">[[&username]]</small>
  [[/display_name]]
  [[^display_name]]
    <span class="username mention-result--username">[[&username]]</span>
  [[/display_name]]
  [[#is_teammate]][[^is_team]]
    <span class="aui-lozenge aui-lozenge-complete aui-lozenge-subtle mention-result--lozenge">teammate</span>
  [[/is_team]][[/is_teammate]]
</span>
      </script>
    
      <script id="mention-call-to-action" type="text/html">
        
[[^query]]
<li class="bb-typeahead-item">Begin typing to search for a user</li>
[[/query]]
[[#query]]
<li class="bb-typeahead-item">Continue typing to search for a user</li>
[[/query]]

      </script>
    
      <script id="mention-no-results" type="text/html">
        
[[^searching]]
<li class="bb-typeahead-item">Found no matching users for <em>[[query]]</em>.</li>
[[/searching]]
[[#searching]]
<li class="bb-typeahead-item bb-typeahead-searching">Searching for <em>[[query]]</em>.</li>
[[/searching]]

      </script>
    
      <script id="emoji-result" type="text/html">
        
<span class="emoji-result">
  <span class="emoji-result--avatar">
    <img class="emoji" src="[[src]]">
  </span>
  <span class="name emoji-result--name">[[&name]]</span>
</span>

      </script>
    
      <script id="repo-typeahead-result" type="text/html">
        <span class="aui-avatar aui-avatar-project aui-avatar-xsmall">
  <span class="aui-avatar-inner">
    <img src="[[avatar]]">
  </span>
</span>
<span class="owner">[[&owner]]</span>/<span class="slug">[[&slug]]</span>

      </script>
    
      <script id="share-form-template" type="text/html">
        

<div class="error aui-message hidden">
  <span class="aui-icon icon-error"></span>
  <div class="message"></div>
</div>
<form class="aui">
  <table class="widget bb-list aui">
    <thead>
    <tr class="assistive">
      <th class="user">User</th>
      <th class="role">Role</th>
      <th class="actions">Actions</th>
    </tr>
    </thead>
    <tbody>
      <tr class="form">
        <td colspan="2">
          <input type="text" class="text bb-user-typeahead user-or-email"
                 placeholder="Username or email address"
                 autocomplete="off"
                 data-bb-typeahead-focus="false"
                 [[#disabled]]disabled[[/disabled]]>
        </td>
        <td class="actions">
          <button type="submit" class="aui-button aui-button-light" disabled>Add</button>
        </td>
      </tr>
    </tbody>
  </table>
</form>

      </script>
    
      <script id="share-detail-template" type="text/html">
        

[[#username]]
<td class="user
    [[#hasCustomGroups]]custom-groups[[/hasCustomGroups]]"
    [[#error]]data-error="[[error]]"[[/error]]>
  <div title="[[displayName]]">
    <a href="/[[username]]/" class="user">
      <span class="aui-avatar aui-avatar-xsmall">
        <span class="aui-avatar-inner">
          <img src="[[avatar]]">
        </span>
      </span>
      <span>[[displayName]]</span>
    </a>
  </div>
</td>
[[/username]]
[[^username]]
<td class="email
    [[#hasCustomGroups]]custom-groups[[/hasCustomGroups]]"
    [[#error]]data-error="[[error]]"[[/error]]>
  <div title="[[email]]">
    <span class="aui-icon aui-icon-small aui-iconfont-email"></span>
    [[email]]
  </div>
</td>
[[/username]]
<td class="role
    [[#hasCustomGroups]]custom-groups[[/hasCustomGroups]]">
  <div>
    [[#group]]
      [[#hasCustomGroups]]
        <select class="group [[#noGroupChoices]]hidden[[/noGroupChoices]]">
          [[#groups]]
            <option value="[[slug]]"
                [[#isSelected]]selected[[/isSelected]]>
              [[name]]
            </option>
          [[/groups]]
        </select>
      [[/hasCustomGroups]]
      [[^hasCustomGroups]]
      <label>
        <input type="checkbox" class="admin"
            [[#isAdmin]]checked[[/isAdmin]]>
        Administrator
      </label>
      [[/hasCustomGroups]]
    [[/group]]
    [[^group]]
      <ul>
        <li class="permission aui-lozenge aui-lozenge-complete
            [[^read]]aui-lozenge-subtle[[/read]]"
            data-permission="read">
          read
        </li>
        <li class="permission aui-lozenge aui-lozenge-complete
            [[^write]]aui-lozenge-subtle[[/write]]"
            data-permission="write">
          write
        </li>
        <li class="permission aui-lozenge aui-lozenge-complete
            [[^admin]]aui-lozenge-subtle[[/admin]]"
            data-permission="admin">
          admin
        </li>
      </ul>
    [[/group]]
  </div>
</td>
<td class="actions
    [[#hasCustomGroups]]custom-groups[[/hasCustomGroups]]">
  <div>
    <a href="#" class="delete">
      <span class="aui-icon aui-icon-small aui-iconfont-remove">Delete</span>
    </a>
  </div>
</td>

      </script>
    
      <script id="share-team-template" type="text/html">
        

<div class="clearfix">
  <span class="team-avatar-container">
    <span class="aui-avatar aui-avatar-medium">
      <span class="aui-avatar-inner">
        <img src="[[avatar]]">
      </span>
    </span>
  </span>
  <span class="team-name-container">
    [[display_name]]
  </span>
</div>
<p class="helptext">
  
    Existing users are granted access to this team immediately.
    New users will be sent an invitation.
  
</p>
<div class="share-form"></div>

      </script>
    
      <script id="scope-list-template" type="text/html">
        <ul class="scope-list">
  [[#scopes]]
    <li class="scope-list--item">
      <span class="scope-list--icon aui-icon aui-icon-small [[icon]]"></span>
      <span class="scope-list--description">[[description]]</span>
    </li>
  [[/scopes]]
</ul>

      </script>
    
      <script id="source-changeset" type="text/html">
        

<a href="/MartinAppo/diploaf-2017/src/[[raw_node]]/[[path]]?at=master"
    class="[[#selected]]highlight[[/selected]]"
    data-hash="[[node]]">
  [[#author.username]]
    <span class="aui-avatar aui-avatar-xsmall">
      <span class="aui-avatar-inner">
        <img src="[[author.avatar]]">
      </span>
    </span>
    <span class="author" title="[[raw_author]]">[[author.display_name]]</span>
  [[/author.username]]
  [[^author.username]]
    <span class="aui-avatar aui-avatar-xsmall">
      <span class="aui-avatar-inner">
        <img src="https://d301sr5gafysq2.cloudfront.net/4154430a88b8/img/default_avatar/user_blue.svg">
      </span>
    </span>
    <span class="author unmapped" title="[[raw_author]]">[[author]]</span>
  [[/author.username]]
  <time datetime="[[utctimestamp]]" data-title="true">[[utctimestamp]]</time>
  <span class="message">[[message]]</span>
</a>

      </script>
    
      <script id="embed-template" type="text/html">
        

<form class="aui inline-dialog-embed-dialog">
  <label for="embed-code-[[dialogId]]">Embed this source in another page:</label>
  <input type="text" readonly="true" value="&lt;script src=&quot;[[url]]&quot;&gt;&lt;/script&gt;" id="embed-code-[[dialogId]]" class="embed-code">
</form>

      </script>
    
  </div>




  
  


<script nonce="">
  window.__initial_state__ = {"section": {"repository": {"connectActions": [], "cloneProtocol": "https", "currentRepository": {"scm": "git", "website": "", "name": "diploaf-2017", "language": "", "description": "", "links": {"clone": [{"href": "https://bitbucket.org/MartinAppo/diploaf-2017.git", "name": "https"}, {"href": "git@bitbucket.org:MartinAppo/diploaf-2017.git", "name": "ssh"}], "self": {"href": "https://bitbucket.org/!api/2.0/repositories/MartinAppo/diploaf-2017"}, "html": {"href": "https://bitbucket.org/MartinAppo/diploaf-2017"}, "avatar": {"href": "https://bytebucket.org/ravatar/%7Bc3518bc4-b447-4571-8f36-b7eb88577f94%7D?ts=default"}}, "mainbranch": {"name": "master"}, "full_name": "MartinAppo/diploaf-2017", "owner": {"username": "MartinAppo", "website": null, "display_name": "Martin Appo", "account_id": "557058:fd109538-456c-440f-9eae-22a22a2f28a8", "links": {"self": {"href": "https://bitbucket.org/!api/2.0/users/MartinAppo"}, "html": {"href": "https://bitbucket.org/MartinAppo/"}, "avatar": {"href": "https://bitbucket.org/account/MartinAppo/avatar/"}}, "created_on": "2014-02-13T08:34:25.561066+00:00", "is_staff": false, "location": null, "type": "user", "uuid": "{5f380218-64ac-4571-a87e-d79700f27317}"}, "type": "repository", "slug": "diploaf-2017", "is_private": false, "uuid": "{c3518bc4-b447-4571-8f36-b7eb88577f94}"}, "menuItems": [{"analytics_label": "repository.overview", "is_client_link": false, "icon_class": "icon-overview", "badge_label": null, "weight": 100, "url": "/MartinAppo/diploaf-2017/overview", "tab_name": "overview", "can_display": true, "label": "Overview", "type": "menu_item", "anchor": true, "analytics_payload": {}, "matching_url_prefixes": [], "target": "_self", "id": "repo-overview-link", "icon": "icon-overview"}, {"analytics_label": "repository.source", "is_client_link": false, "icon_class": "icon-source", "badge_label": null, "weight": 200, "url": "/MartinAppo/diploaf-2017/src", "tab_name": "source", "can_display": true, "label": "Source", "type": "menu_item", "anchor": true, "analytics_payload": {}, "matching_url_prefixes": ["/diff", "/history-node"], "target": "_self", "id": "repo-source-link", "icon": "icon-source"}, {"analytics_label": "repository.commits", "is_client_link": false, "icon_class": "icon-commits", "badge_label": null, "weight": 300, "url": "/MartinAppo/diploaf-2017/commits/", "tab_name": "commits", "can_display": true, "label": "Commits", "type": "menu_item", "anchor": true, "analytics_payload": {}, "matching_url_prefixes": [], "target": "_self", "id": "repo-commits-link", "icon": "icon-commits"}, {"analytics_label": "repository.branches", "is_client_link": false, "icon_class": "icon-branches", "badge_label": null, "weight": 400, "url": "/MartinAppo/diploaf-2017/branches/", "tab_name": "branches", "can_display": true, "label": "Branches", "type": "menu_item", "anchor": true, "analytics_payload": {}, "matching_url_prefixes": [], "target": "_self", "id": "repo-branches-link", "icon": "icon-branches"}, {"analytics_label": "repository.pullrequests", "is_client_link": false, "icon_class": "icon-pull-requests", "badge_label": null, "weight": 500, "url": "/MartinAppo/diploaf-2017/pull-requests/", "tab_name": "pullrequests", "can_display": true, "label": "Pull requests", "type": "menu_item", "anchor": true, "analytics_payload": {}, "matching_url_prefixes": [], "target": "_self", "id": "repo-pullrequests-link", "icon": "icon-pull-requests"}, {"analytics_label": "repository.downloads", "is_client_link": false, "icon_class": "icon-downloads", "badge_label": null, "weight": 800, "url": "/MartinAppo/diploaf-2017/downloads/", "tab_name": "downloads", "can_display": true, "label": "Downloads", "type": "menu_item", "anchor": true, "analytics_payload": {}, "matching_url_prefixes": [], "target": "_self", "id": "repo-downloads-link", "icon": "icon-downloads"}], "bitbucketActions": [{"analytics_label": "repository.clone", "is_client_link": false, "icon_class": "icon-clone", "badge_label": null, "weight": 100, "url": "#clone", "tab_name": "clone", "can_display": true, "label": "<strong>Clone<\/strong> this repository", "type": "menu_item", "anchor": true, "analytics_payload": {}, "matching_url_prefixes": [], "target": "_self", "id": "repo-clone-button", "icon": "icon-clone"}, {"analytics_label": "repository.compare", "is_client_link": false, "icon_class": "aui-icon-small aui-iconfont-devtools-compare", "badge_label": null, "weight": 400, "url": "/MartinAppo/diploaf-2017/branches/compare", "tab_name": "compare", "can_display": true, "label": "<strong>Compare<\/strong> branches or tags", "type": "menu_item", "anchor": true, "analytics_payload": {}, "matching_url_prefixes": [], "target": "_self", "id": "repo-compare-link", "icon": "aui-icon-small aui-iconfont-devtools-compare"}, {"analytics_label": "repository.fork", "is_client_link": false, "icon_class": "icon-fork", "badge_label": null, "weight": 500, "url": "/MartinAppo/diploaf-2017/fork", "tab_name": "fork", "can_display": true, "label": "<strong>Fork<\/strong> this repository", "type": "menu_item", "anchor": true, "analytics_payload": {}, "matching_url_prefixes": [], "target": "_self", "id": "repo-fork-link", "icon": "icon-fork"}], "activeMenuItem": "source"}}, "global": {"needs_marketing_consent": false, "features": {"connect-proxy-outbound": true, "source-browser-file-filter": true, "exp-share-to-invite-variation": false, "clone-in-xcode": true, "gdpr-marketing-consent": true, "allow-pullrequest-live-reviewers": true, "connect-v5": true, "mobile-nav": true, "deployments": true, "fe_word_diff": true, "merge-api-shorter-lock-timeout": true, "use-moneybucket": true, "src-lastmod-for-dirs": true, "app-passwords": true, "nav-add-file": false, "show-billing-errors": true, "trello-boards": true, "cache-ref-adverts": true, "bitbucket-chats-integration": true}, "locale": "en", "geoip_country": null, "targetFeatures": {"new-code-review-experiment": false, "merge-api-shorter-lock-timeout": true, "show-billing-errors": true, "show-guidance-message": true, "search-satisfaction": true, "bitbucket-chats-integration": true, "source-browser-file-filter": true, "cache-ref-adverts": true, "mobile-nav": true, "deployments": true, "fe_word_diff": true, "clonebundles": true, "use-moneybucket": true, "pride-logo": false, "diff-api-renames": false, "connect-proxy-outbound": true, "clone-in-xcode": true, "gdpr-marketing-consent": true, "connect-v5": true, "trello-boards": true, "atlassian-editor": true, "src-lastmod-for-dirs": true, "new-source-browser": true, "exp-new-user-survey": true, "evolution": false, "app-passwords": true, "allow-pullrequest-live-reviewers": true}, "isFocusedTask": false, "browser_monitoring": true, "targetUser": {"username": "MartinAppo", "website": null, "display_name": "Martin Appo", "account_id": "557058:fd109538-456c-440f-9eae-22a22a2f28a8", "links": {"self": {"href": "https://bitbucket.org/!api/2.0/users/MartinAppo"}, "html": {"href": "https://bitbucket.org/MartinAppo/"}, "avatar": {"href": "https://bitbucket.org/account/MartinAppo/avatar/"}}, "created_on": "2014-02-13T08:34:25.561066+00:00", "is_staff": false, "location": null, "type": "user", "uuid": "{5f380218-64ac-4571-a87e-d79700f27317}"}, "is_mobile_user_agent": false, "flags": [], "site_message": "", "isNavigationOpen": true, "path": "/MartinAppo/diploaf-2017/src/master/hardware_module/src/hardware_module/comport_mainboard.py", "focusedTaskBackButtonUrl": null, "whats_new_feed": "https://bitbucket.org/blog/wp-json/wp/v2/posts?context=embed&per_page=6&orderby=date&order=desc"}};
  window.__settings__ = {"MARKETPLACE_TERMS_OF_USE_URL": null, "JIRA_ISSUE_COLLECTORS": {"source-browser": {"url": "https://bitbucketfeedback.atlassian.net/s/d41d8cd98f00b204e9800998ecf8427e-T/-tqnsjm/b/20/a44af77267a987a660377e5c46e0fb64/_/download/batch/com.atlassian.jira.collector.plugin.jira-issue-collector-plugin:issuecollector/com.atlassian.jira.collector.plugin.jira-issue-collector-plugin:issuecollector.js?locale=en-US&collectorId=c19c2ff6", "id": "c19c2ff6"}, "code-review": {"url": "https://bitbucketfeedback.atlassian.net/s/d41d8cd98f00b204e9800998ecf8427e-T/-4bqv2z/b/20/a44af77267a987a660377e5c46e0fb64/_/download/batch/com.atlassian.jira.collector.plugin.jira-issue-collector-plugin:issuecollector/com.atlassian.jira.collector.plugin.jira-issue-collector-plugin:issuecollector.js?locale=en-US&collectorId=bb066400", "id": "bb066400"}}, "CANON_URL": "https://bitbucket.org", "CONSENT_HUB_FRONTEND_BASE_URL": "https://preferences.atlassian.com", "API_CANON_URL": "https://api.bitbucket.org", "SOCIAL_AUTH_ATLASSIANID_LOGOUT_URL": "https://id.atlassian.com/logout", "EMOJI_STANDARD_BASE_URL": "https://pf-emoji-service.prod.public.atl-paas.net/emoji/"};
  window.__webpack_nonce__ = '';
</script>

<script src="https://d301sr5gafysq2.cloudfront.net/4154430a88b8/jsi18n/en/djangojs.js" nonce=""></script>

  <script src="https://d301sr5gafysq2.cloudfront.net/4154430a88b8/dist/webpack/locales/en.js" nonce=""></script>

<script src="https://d301sr5gafysq2.cloudfront.net/4154430a88b8/dist/webpack/vendor.js" nonce=""></script>
<script src="https://d301sr5gafysq2.cloudfront.net/4154430a88b8/dist/webpack/app.js" nonce=""></script>


<script async src="https://www.google-analytics.com/analytics.js" nonce=""></script>

<script nonce="" type="text/javascript">window.NREUM||(NREUM={});NREUM.info={"beacon":"bam.nr-data.net","queueTime":0,"licenseKey":"a2cef8c3d3","agent":"","transactionName":"Z11RZxdWW0cEVkYLDV4XdUYLVEFdClsdAAtEWkZQDlJBGgRFQhFMQl1DXFcZQ10AQkFYBFlUVlEXWEJHAA==","applicationID":"1841284","errorBeacon":"bam.nr-data.net","applicationTime":643}</script>
</body>
</html>