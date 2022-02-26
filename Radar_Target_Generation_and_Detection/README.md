# Radar Target Generation and Detection

## Implementation steps for the 2D CFAR process

Lines 143-209 in script Radar_Target_Generation_and_Detection.m

The 2D constant false alarm rate (CFAR), when applied to the results of the 2D FFT, uses a dynamic threshold set by the noise level in the vicinity of the cell under test (CUT). The key steps are as follows:

1. Loop over all cells in the range and doppler dimensions, starting and ending at indices which leave appropriate margins
2. Slice the training cells (and exclude the guard cells) surrounding the CUT
3. Convert the training cell values from decibels (dB) to power, to linearize
4. Find the mean noise level among the training cells
5. Convert this average value back from power to dB
6. Add the offset (in dB) to set the dynamic threshold
7. Apply the threshold and store the result in a binary array of the same dimensions as the range doppler map (RDM)

```matlab
for i = Tr + Gr + 1 : Nr/2 - Tr - Gr
    for j = Td + Gd + 1 : Nd - Td - Gd
        training = RDM(i - Tr - Gr : i + Tr + Gr, j - Td - Gd : j + Td + Gd);
        training = db2pow(training);
        training = sum(training) / Tcell;
        training = pow2db(training);
        threshold = training + offset;
        CUT = RDM(i, j);
        if CUT > threshold
            RDM(i, j) = 1;
        end
    end
end
```

## Selection of training cells, guard cells, and offset

Lines 149-162 in script Radar_Target_Generation_and_Detection.m

The values below were hand selected. I chose a rectangular window with the major dimension along the range cells. This produced better filtered results from the given RDM. Choosing the right value for offset was key to isolating the simulated target and avoiding false positives. Finally, I precalculated the (Tcell) value to avoid a performance hit in the nested loop.

```matlab
Tr = 12;
Td = 3;
% *%TODO* :
%Select the number of Guard Cells in both dimensions around the Cell under 
%test (CUT) for accurate estimation
Gr = 4;
Gd = 1;
% *%TODO* :
% offset the threshold by SNR value in dB
offset = 15;
% *%TODO* :
%Create a vector to store noise_level for each iteration on training cells
noise_level = zeros(1,1);
Tcell = (2*Tr+2*Gr+1)*(2*Td+2*Gd+1) - (2*Gr+1)*(2*Gd+1);
```

## Steps taken to suppress the non-thresholded cells at the edges

Line 202 in script Radar_Target_Generation_and_Detection.m

```matlab

RDM(RDM~=0 & RDM~=1) = 0;
```