 aviobj =avifile('video.avi','Compression','None','fps',15);
for i = 1 : 382
  % load image
  Im = (imread(['DATA/',int2str(i), '.jpg'])); 
   
refreshdata
imshow(Im);
imi = getframe(gca);
aviobj = addframe(aviobj,imi);
end
aviobj = close(aviobj);