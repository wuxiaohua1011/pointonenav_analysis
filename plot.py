import gmplot
import pandas as pd 
from gmplot import GoogleMapPlotter
# We subclass this just to change the map type
class CustomGoogleMapPlotter(GoogleMapPlotter):
    def __init__(self, center_lat, center_lng, zoom, apikey='',
                 map_type='satellite'):
        super().__init__(center_lat, center_lng, zoom, apikey)

        self.map_type = map_type
        assert(self.map_type in ['roadmap', 'satellite', 'hybrid', 'terrain'])

    def write_map(self,  f):
        f.write('\t\tvar centerlatlng = new google.maps.LatLng(%f, %f);\n' %
                (self.center[0], self.center[1]))
        f.write('\t\tvar myOptions = {\n')
        f.write('\t\t\tzoom: %d,\n' % (self.zoom))
        f.write('\t\t\tcenter: centerlatlng,\n')

        # This is the only line we change
        f.write('\t\t\tmapTypeId: \'{}\'\n'.format(self.map_type))


        f.write('\t\t};\n')
        f.write(
            '\t\tvar map = new google.maps.Map(document.getElementById("map_canvas"), myOptions);\n')
        f.write('\n')

file_name = "gps_1"
df = pd.read_csv(f"./{file_name}.csv")
latitude_list = list(df.latitude)
longitude_list = list(df.longitude)
  
gmap3 = CustomGoogleMapPlotter(37.915371,
                                -122.334168, 20, apikey="")
  
# scatter method of map object 
# scatter points on the google map
gmap3.scatter( latitude_list, longitude_list, '#FF0000',
                              size = 0.01, marker=False )

gmap3.draw( f"./{file_name}.html" )