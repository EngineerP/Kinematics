function [V,F,C] = canonical_bone();
  V = [
    0 -1 -1
    0  1 -1
    0  1  1
    0 -1  1
    1  0  0];
  F = [1 2 3;1 3 4;2 1 5;3 2 5;4 3 5;1 4 5];
  C = [
    0 1 1
    0 1 1
    1 1 0
    0 1 0
    0 0 1
    1 0 1];
end
