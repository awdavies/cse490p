function b = floor_contact(foot, contact)
for i = 1:length(contact)
    if (contact(i).obj2 == foot)
          b = true;
          return;
    end
end

b = false;
