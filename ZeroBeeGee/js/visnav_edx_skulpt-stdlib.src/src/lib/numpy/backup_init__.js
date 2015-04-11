var $builtinmodule = function(name)
{
  var mod = {};
  
  /**
   * ndarray class
   */
  var ndarray = function($gbl, $loc) {
    $loc.__init__ = new Sk.builtin.func(function(self, shape, buffer) {
      Sk.builtin.pyCheckArgs('numpy.ndarray.__init__', arguments, 2, 3);
      
      self.$d.mp$ass_subscript(new Sk.builtin.str('shape'), shape);
      self.$d.mp$ass_subscript(new Sk.builtin.str('ndim'), new Sk.builtin.nmber(shape.v.length, Sk.builtin.nmber.int$));
      self.data = buffer;
    });
  };
  
  mod.ndarray = Sk.misceval.buildClass(mod, ndarray, 'ndarray', []);
  
  function compare(arr1, arr2) {
    if(arr1.length != arr2.length) return false;
    
    var idx;
    for(idx = 0; idx < arr1.length; idx++) {
      if(arr1[idx] != arr2[idx]) return false;
    }
    
    return true;
  }
  
  function getShapeFromList(l) {
    if(l.tp$name !== undefined && l.tp$name === 'list') {
      if(l.v.length == 0) {
        return [0]
      }
    
      var shape = getShapeFromList(l.v[0]);
    
      var idx;
      for(idx = 1; idx < l.v.length; ++idx) {
        if(!compare(shape, getShapeFromList(l.v[idx]))) {
          console.log("inner dimensions, don't agree");
          // TODO: error
        }
      }
      
      return [l.v.length].concat(shape);
    } else if(l.skType !== undefined) {
      return [];
    }
    
    // TODO: error
    console.log("no array, nor number");
  }  
  
  /**
   * array creation functions
   */
  mod.array = new Sk.builtin.func(function(object) {
    Sk.builtin.pyCheckArgs('array', arguments, 1);
    
    return Sk.misceval.callsim(mod.ndarray, Sk.builtin.tuple(getShapeFromList(object)), object);
  });
  
  return mod;
}
