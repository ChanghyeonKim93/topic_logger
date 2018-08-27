function skew_mat = vect_skew_kch(vect)
% Author : Changhyeon Kim, ICSL, Seoul National University.
% Date   : 2018-08-23
% Funtionality : convert 3-d vector into skew matrix (3x3).
% Input  : 3-d vector
% Output : Skew matrix (3x3).

skew_mat = zeros(3);
skew_mat(1,2) = -vect(3);
skew_mat(1,3) = vect(2);
skew_mat(2,1) = vect(3);
skew_mat(2,3) = -vect(1);
skew_mat(3,1) = -vect(2);
skew_mat(3,2) = vect(1);

end