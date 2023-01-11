def limit_generator(starting_x, starting_y, target_x, target_y):
    limit_x = abs(target_x - starting_x)
    limit_y = abs(target_y - starting_y)
    
    return buffer_generator(limit_x, limit_y)

def buffer_generator(limit_x, limit_y):
    if limit_x < 80:
        buffer_x = 100 - limit_x
    else:
        buffer_x = 20
    
    if limit_y < 80:
        buffer_y = 100 - limit_y
    else:
        buffer_y = 20
    
    new_limit_x = limit_x + buffer_x
    new_limit_y = limit_y + buffer_y
    
    return [new_limit_x, new_limit_y]

#before flight call limit_generator
# in every step check
#abs(target.x - current.x) > limit_x
#abs(target.y - current.y) > limit_y
