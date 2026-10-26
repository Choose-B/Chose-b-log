/* global hexo */
'use strict';

// Fluid's synchronous containers truncate their body at an async mathjax tag.
// Keep the theme renderers and markup, but await nested tags before rendering.
const patched = new WeakSet();
hexo.extend.filter.register('before_generate', () => {
  if (hexo.config.theme !== 'fluid') return;
  for (const name of ['fold', 'note']) {
    const current = hexo.extend.tag.env.getExtension(name);
    if (!current || patched.has(current)) continue;
    // Nunjucks keeps a parser extension list: replacing by name alone leaves
    // the old synchronous parser paired with the new asynchronous renderer.
    hexo.extend.tag.unregister(name);
    hexo.extend.tag.register(name, current.fn, { ends: true, async: true });
    patched.add(hexo.extend.tag.env.getExtension(name));
  }
}, 1);
