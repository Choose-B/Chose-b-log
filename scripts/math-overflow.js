/* global hexo */
'use strict';

// 在所有页面注入长公式溢出处理的样式与脚本（与主题无关，换主题也生效）
// 文件位于 source/css/math-overflow.css 与 source/js/math-overflow.js
const urlFor = hexo.extend.helper.get('url_for').bind(hexo);

hexo.extend.injector.register('head_end', () =>
  `<link rel="stylesheet" href="${urlFor('/css/math-overflow.css')}">`
);

hexo.extend.injector.register('body_end', () =>
  `<script src="${urlFor('/js/math-overflow.js')}" defer></script>`
);
