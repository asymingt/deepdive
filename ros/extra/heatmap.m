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
ootx(3).phase = -0.0867857;
ootx(3).tilt = 0.127538;
ootx(3).gib_phase = 1.91645;
ootx(3).gib_mag = 0.0411464;
ootx(3).curve = 0.0519228;
ootx(4).phase = 0.198872;
ootx(4).tilt = 0.0124835;
ootx(4).gib_phase = -1.23938;
ootx(4).gib_mag = 0.326461;
ootx(4).curve = -0.00920232;

# Print the map
n = 64;
s = [-0.2 0.2];
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
    % REFERENCE
    Xa(i, j) = atan2(x - (ootx(3).tilt + ootx(3).curve * y) * y, z);
    Ya(i, j) = atan2(y - (ootx(4).tilt + ootx(4).curve * x) * x, z);
    Xa(i, j) = Xa(i, j) - ootx(3).phase - ootx(3).gib_mag * sin(Xa(i, j) + ootx(3).gib_phase);
    Ya(i, j) = Ya(i, j) - ootx(4).phase - ootx(4).gib_mag * sin(Ya(i, j) + ootx(4).gib_phase);
    Xa(i, j) = Xa(i, j) - ideal(1);
    Ya(i, j) = Ya(i, j) - ideal(2);
    
    % ESTIMATE
    Xb(i, j) = atan2(x - ootx(1).tilt * y + ootx(1).curve * (y * y * y * y), z);
    Yb(i, j) = atan2(y - ootx(2).tilt * x + ootx(2).curve * (x * x * x * x), z);
    Xb(i, j) = Xb(i, j) - ootx(1).phase - ootx(1).gib_mag * sin(Xb(i, j) + ootx(1).gib_phase);
    Yb(i, j) = Yb(i, j) - ootx(2).phase - ootx(2).gib_mag * sin(Yb(i, j) + ootx(2).gib_phase);
    Xb(i, j) = Xb(i, j) - ideal(1);
    Yb(i, j) = Yb(i, j) - ideal(2);
  endfor
endfor
subplot(2, 2, 1);
imagesc(Xb, s);
axis equal;
colorbar;
title('AXIS 0 ESTIMATE');
subplot(2, 2, 2);
imagesc(Yb, s);
axis equal;
colorbar;
title('AXIS 1 ESTIMATE');
subplot(2, 2, 3);
imagesc(Xa, s);
axis equal;
colorbar;
title('AXIS 0 REFERENCE');
subplot(2, 2, 4);
imagesc(Ya, s);
axis equal;
colorbar;
title('AXIS 1 REFRENCE');