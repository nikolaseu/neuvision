function generateTextures(nbits)

    if nargin<1,
        nbits = 11;
    end
    
    nrows = 2^nbits;
    
    grayci = false(nrows,nbits+1);
    grayci(:,1) = true;
    for i=0:nrows-1,
        binaryc = dec2gray(i);
        for j=1:numel(binaryc),
            if binaryc(end-j+1),
                grayci(i+1,j+1) = 1;
            end
        end
    end
    % repito patrones
    grayci = repeatCols(grayci);
    imwrite(grayci, fullfile('..', 'res', 'pattern', sprintf('gray.%g.bmp', nrows)));
    imwrite(~grayci, fullfile('..', 'res', 'pattern', sprintf('gray.%g.inv.bmp', nrows)));

    grayci = false(nrows,nbits+1);
    grayci(:,1) = true;
    for i=0:nrows-1,
        binaryc = dec2bin(i) == '1';
        for j=1:numel(binaryc),
            if binaryc(end-j+1),
                grayci(i+1,j+1) = 1;
            end
        end
    end
    % repito patrones
    grayci = repeatCols(grayci);
    imwrite(grayci, fullfile('..', 'res', 'pattern', sprintf('binary.%g.bmp', nrows)));
    imwrite(~grayci, fullfile('..', 'res', 'pattern', sprintf('binary.%g.inv.bmp', nrows)));
    
    % grayci = uint8(nrows,1);
    step = 360/nrows;
    t = [0 : step : 360-step]';
    grayci = uint8(255 * (sind(t) + 1)/2);
    imwrite(grayci, fullfile('..', 'res', 'pattern', sprintf('sin.%g.png', nrows)));


function gray = dec2gray(num)
    bnum = dec2bin(num) == '1';
    shiftednum = [0 bnum(1:end-1)];
    gray = xor(bnum, shiftednum);
    
function grayci = repeatCols(grayci)
    % repito patrones
    grayci2 = grayci;
    nrows = size(grayci,1);
    ncols = size(grayci,2);
    grayci = false(nrows, 2*ncols);
    for j=0:ncols-1,
        grayci(:,2*j+1) = grayci2(:,j+1);
        grayci(:,2*j+2) = grayci2(:,j+1);
    end
