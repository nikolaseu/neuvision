function generateProjectorImages(nbits)

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
    for i=1:size(grayci, 2),
        img = false(768,1024);
        for r=0:nrows-1,
            img(:,r+1) = grayci(r+1,i)';
        end
        imwrite(img, fullfile('..', 'res', 'projector', sprintf('gray_%02i.png', i-1)));
        imwrite(~img, fullfile('..', 'res', 'projector', sprintf('gray_%02i_inv.png', i-1)));
    end

    % to do
    return;

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
    imwrite(grayci, fullfile('..', 'res', 'projector', sprintf('binary_%02i.png', nrows)));
    imwrite(~grayci, fullfile('..', 'res', 'projector', sprintf('binary_%02i_inv.png', nrows)));
    
    % grayci = uint8(nrows,1);
    step = 360/nrows;
    t = [0 : step : 360-step]';
    grayci = uint8(255 * (sind(t) + 1)/2);
    imwrite(grayci, fullfile('..', 'res', 'pattern', sprintf('sin.%g.png', nrows)));


function gray = dec2gray(num)
    bnum = dec2bin(num) == '1';
    shiftednum = [0 bnum(1:end-1)];
    gray = xor(bnum, shiftednum);
