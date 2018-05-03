% Lighthouse parameters
ootx = struct;

% BEFORE
ootx(1).phase = 0.0302429;
ootx(1).tilt = 0.00762177;
ootx(1).gib_phase = 1.71582;
ootx(1).gib_mag = 0.0114594;
ootx(1).curve = -0.000842571;
ootx(2).phase = 0.0285797;
ootx(2).tilt = -0.012146;
ootx(2).gib_phase = -0.440918;
ootx(2).gib_mag = -0.00743484;
ootx(2).curve = 0.00172043;

% AFTER
ootx(3).phase = 0.0282381;
ootx(3).tilt = 0.0156368;
ootx(3).gib_phase = 2.1669;
ootx(3).gib_mag = 0.0389253;
ootx(3).curve = 0.0080333;
ootx(4).phase = 0.0701366;
ootx(4).tilt = 0.0281246;
ootx(4).gib_phase = 0.071969;
ootx(4).gib_mag = -0.142944;
ootx(4).curve = -0.0151812;

# Print the map
n = 64;
Xa = zeros(n, n);
Ya = zeros(n, n);
Xb = zeros(n, n);
Yb = zeros(n, n);
for i = 1:n
  for j = 1:n
    % Get the angle
    ideal(1) = 2 * (i/n - 0.5) * 60 * pi / 180;
    ideal(2) = 2 * (j/n - 0.5) * 60 * pi / 180;
    % Get a position on the image plane
    x = tan(ideal(1));
    y = tan(ideal(2));
    z = 1.0;
    % Perturb the AFTER measurement to derive a reference
    Xa(i, j) = atan2(x - (ootx(3).tilt + ootx(3).curve * y) * y, z);
    Ya(i, j) = atan2(y - (ootx(4).tilt + ootx(4).curve * x) * x, z);
    Xa(i, j) = Xa(i, j) - ootx(3).phase - ootx(3).gib_mag * sin(Xa(i, j) + ootx(3).gib_phase);
    Ya(i, j) = Ya(i, j) - ootx(4).phase - ootx(4).gib_mag * sin(Ya(i, j) + ootx(4).gib_phase);
    Xa(i, j) = Xa(i, j) - ideal(1);
    Ya(i, j) = Ya(i, j) - ideal(2);
    % Perturb the BEFORE measurement
    Xb(i, j) = ideal(1) - ootx(1).phase - ootx(1).gib_mag * sin(ideal(1) + ootx(1).gib_phase);
    Yb(i, j) = ideal(2) - ootx(2).phase - ootx(2).gib_mag * sin(ideal(2) + ootx(2).gib_phase);
    x = tan(Xb(i, j));
    y = tan(Yb(i, j));
    z = 1.0;
    Xb(i, j) = atan2(x + (ootx(1).tilt + ootx(1).curve * y) * y, z);
    Yb(i, j) = atan2(y + (ootx(2).tilt + ootx(2).curve * x) * x, z);
    Xb(i, j) = Xb(i, j) - ideal(1);
    Yb(i, j) = Yb(i, j) - ideal(2);

  endfor
endfor
subplot(2, 2, 1);
imagesc(Xb);
axis equal;
colorbar;
title('AXIS 0 BEFORE');
subplot(2, 2, 2);
imagesc(Yb);
axis equal;
colorbar;
title('AXIS 1 BEFORE');
subplot(2, 2, 3);
imagesc(Xa);
axis equal;
colorbar;
title('AXIS 0 AFTER');
subplot(2, 2, 4);
imagesc(Ya);
axis equal;
colorbar;
title('AXIS 1 AFTER');