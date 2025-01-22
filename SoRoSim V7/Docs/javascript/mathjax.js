window.MathJax = {
    tex: {
      inlineMath: [["\(", "\)"]],
      displayMath: [["\[", "\]"]],
      processEscapes: true,
      processEnvironments: true
    },
    options: {
      ignoreHtmlClass: ".*|",
      processHtmlClass: "arithmatex"
    },
    loader: {load: ['[tex]/amsmath']},
    tex: {packages: {'[+]': ['color']}}
  };
  
  document$.subscribe(() => { 
    MathJax.startup.output.clearCache()
    MathJax.typesetClear()
    MathJax.texReset()
    MathJax.typesetPromise()
  })