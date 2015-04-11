class ndarray:
  def __init__(self, shape, buffer):
    self.shape = shape;
    self.ndim = len(shape);
    self._data = buffer;
    
  def __add__(self, other):
      from numpy_internal_math import add;
      
      return add(self, other);
      
  def __str__(self):
      from numpy_internal_math import array_str;
      
      return array_str(self);
      

def _inferShapeFromList(l):
  if type(l) is list:
    if len(l) == 0:
      return (0,)
    
    shape = _inferShapeFromList(l[0])
    
    for i in range(1, len(l)):
      if shape != _inferShapeFromList(l[i]):
        raise ValueError()
    
    return (len(l),) + shape
  else:
    return tuple()

def array(obj):
  return ndarray(_inferShapeFromList(obj), obj)
  
