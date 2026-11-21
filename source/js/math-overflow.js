/*
 * 给超出正文宽度的行内 MathJax 公式加上 .math-scroll 类，使其横向滚动。
 * 样式见 /css/math-overflow.css。块级（display）公式由 CSS 直接处理。
 */
(function () {
  'use strict';

  var SELECTOR = '.markdown-body mjx-container[jax="SVG"]:not([display="true"])';
  var CLASS = 'math-scroll';

  // 找到最近的块级祖先，返回其内容区宽度
  function availableWidth(el) {
    var p = el.parentElement;
    while (p && getComputedStyle(p).display.indexOf('inline') === 0) {
      p = p.parentElement;
    }
    if (!p) return Infinity;
    var cs = getComputedStyle(p);
    return p.clientWidth - parseFloat(cs.paddingLeft) - parseFloat(cs.paddingRight);
  }

  function update() {
    var list = document.querySelectorAll(SELECTOR);
    var flags = [];
    var i;
    // 先统一读取，再统一写入，避免反复触发重排
    for (i = 0; i < list.length; i++) {
      var svg = list[i].querySelector('svg');
      var w = svg ? svg.getBoundingClientRect().width : 0;
      flags.push(w > 0 && w > availableWidth(list[i]) + 1);
    }
    for (i = 0; i < list.length; i++) {
      list[i].classList.toggle(CLASS, flags[i]);
    }
  }

  var pending = false;
  function schedule() {
    if (pending) return;
    pending = true;
    requestAnimationFrame(function () {
      pending = false;
      update();
    });
  }

  function init() {
    update();
    window.addEventListener('resize', schedule);
    window.addEventListener('load', schedule);
    // 折叠块（fold）展开时，里面的公式从隐藏变为可见，需要重新判断
    if ('ResizeObserver' in window) {
      var ro = new ResizeObserver(schedule);
      var list = document.querySelectorAll(SELECTOR);
      for (var i = 0; i < list.length; i++) ro.observe(list[i]);
    }
  }

  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', init);
  } else {
    init();
  }
})();
