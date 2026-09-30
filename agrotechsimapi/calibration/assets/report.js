(() => {
  const words = [
    ['уровень не успокоился; попадание в полосу не засчитано','not settled; in-band time not credited'],
    ['после успокоения: в полосе','after settling: in band'],
    ['средняя ошибка','mean error'],['Чёрная — средняя','Black — mean of'],
    ['сглаживание 0,25 с','0.25 s smoothing'],['3 пролёта в каждую сторону','3 flights per direction'],
    ['средняя в коридоре','mean in band'],['средняя в полосе','mean in band'],
    ['в целевом коридоре','in target band'],['средняя достигла','mean reached'],
    ['цель не достигнута','target not reached'],['приход не определён','arrival unavailable'],
    ['Высота над стартом, м','Height above launch, m'],['Курс, °','Yaw, °'],
    ['Положительная','Positive'],['Отрицательная','Negative'],
    ['Вперёд','Forward'],['Назад','Reverse'],['вперёд','forward'],['назад','reverse'],
    ['успокоение','settling'],['выход на уровень','settling time'],['уровень','level'],
    ['удержание','hold'],['в полосе','in band'],['на пределе','saturated'],
    ['прежняя оценка','legacy score'],['время, с','time, s'],
    ['пролётов','flights'],['пролёт','flight'],['приход','arrival'],
    ['ступень','step'],['допуск','tolerance'],['целей','targets'],['цель','target'],
    ['ошибка','error'],['м/с²','m/s²'],['м/с','m/s'],['мкс','µs'],
    [' с',' s'],[' м',' m']
  ];
  const originals = [];
  const labels = [...document.querySelectorAll('svg[aria-label]')].map(el=>[el,el.getAttribute('aria-label')]);
  const walker = document.createTreeWalker(document.querySelector('main'), NodeFilter.SHOW_TEXT);
  while (walker.nextNode()) {
    const node = walker.currentNode;
    if (!node.parentElement.closest('[data-ru]') && /[А-Яа-яЁё]/.test(node.textContent))
      originals.push([node,node.textContent]);
  }
  function language(lang) {
    document.documentElement.lang = lang;
    document.querySelectorAll('[data-ru]').forEach(el => {el.textContent=el.dataset[lang];});
    originals.forEach(([node,ru]) => {
      let value=ru;
      if(lang==='en') words.forEach(([from,to]) => {value=value.split(from).join(to);});
      node.textContent=value;
    });
    labels.forEach(([el,ru])=>{let value=ru;if(lang==='en') words.forEach(([from,to])=>{value=value.split(from).join(to);});el.setAttribute('aria-label',value);});
    document.querySelectorAll('[data-lang]').forEach(el=>el.setAttribute('aria-pressed',String(el.dataset.lang===lang)));
  }
  const tabs = [...document.querySelectorAll('[data-tab]')];
  function activate(tab) {
    tabs.forEach(el=>{const on=el===tab;el.setAttribute('aria-selected',String(on));el.tabIndex=on?0:-1;document.getElementById(el.dataset.tab).hidden=!on;});
  }
  tabs.forEach((tab,i)=>{
    tab.addEventListener('click',()=>activate(tab));
    tab.addEventListener('keydown',event=>{if(['ArrowLeft','ArrowRight'].includes(event.key)){event.preventDefault();const next=tabs[(i+(event.key==='ArrowRight'?1:tabs.length-1))%tabs.length];activate(next);next.focus();}});
  });
  document.querySelectorAll('[data-lang]').forEach(el=>el.addEventListener('click',()=>language(el.dataset.lang)));
  language(document.documentElement.lang);
})();
