Jordan Heemskerk
V00158837

1. By changing the number of neighbours used, I determined that 6-10 is indeed a good range. If too little are used, the effect creates a very noisy normal reconstruction, as there are no averaging effects from the least squares. If too many points are used, then the surface is reconstructed properly, but at extended computational cost. Also, when too many neighbours are used, some of the high frequency changes are filtered out. See the three attached images to see the effects of 3, 8 and 100 nearest neighbours.

2. Since I am away over reading break, I do not have time to tackle the implicit RBF reconstruction. 
