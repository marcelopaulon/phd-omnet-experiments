package projeto;

import inet.common.geometry.common.SimpleGeographicCoordinateSystem;
import inet.environment.common.PhysicalEnvironment;
import inet.environment.ground.FlatGround;
import inet.node.inet.AdhocHost;
import inet.visualizer.integrated.IntegratedVisualizer;
import inet.mobility.contract.IMobility;
import inet.mobility.static.StationaryMobility;
import inet.networklayer.configurator.ipv4.Ipv4NetworkConfigurator;
import inet.node.ethernet.Eth100M;
import inet.node.inet.SensorNode;
import inet.node.inet.StandardHost;
import inet.node.inet.WirelessHost;
import inet.physicallayer.apskradio.packetlevel.ApskScalarRadioMedium;
import inet.physicallayer.ieee802154.bitlevel.Ieee802154UwbIrRadioMedium;
import inet.physicallayer.ieee802154.packetlevel.Ieee802154NarrowbandScalarRadioMedium;
import inet.physicallayer.ieee80211.packetlevel.Ieee80211ScalarRadioMedium;
import inet.visualizer.integrated.IntegratedMultiVisualizer;
import inet.visualizer.integrated.IntegratedVisualizer;
import inet.physicallayer.contract.packetlevel.IRadioMedium;

network DADCA_ACK_n50_u8_a8000m2_0
{
    parameters:
		int numUAVs = default(1);
    submodules:
        visualizer: IntegratedVisualizer {
            parameters:
                @display("p=100,50");
        }
        coordinateSystem: SimpleGeographicCoordinateSystem {
            parameters:
                @display("p=100,150");
        }
        
        environment: PhysicalEnvironment {
        }
        
        configurator: Ipv4NetworkConfigurator {
            @display("p=0,200");
        }
        radioMedium: <default("Ieee802154NarrowbandScalarRadioMedium")> like IRadioMedium {
            @display("p=200,0");
        }
        quads[numUAVs]: MobileNode {
			
        } 
        
        groundStation: GroundStation {
            @display("i=misc/sensor3");
        }
       
        
    sensors_1: MobileSensorNode {
        @display("i=misc/sensor;p=71.80,7810.86");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=518.25,7789.04");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=944.56,7911.38");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=1670.10,7986.95");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=1951.41,7771.05");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=2441.86,7957.35");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=2820.80,7971.92");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=3718.69,7762.29");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=4780.49,7896.38");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=5166.04,7883.72");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=5788.50,7911.49");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=1141.92,7569.57");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=4193.69,7609.90");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=5381.34,7586.97");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=5722.23,7506.07");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=182.18,7427.21");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=645.65,7408.54");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=1726.18,7459.78");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=2479.86,7361.95");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=2974.30,7427.61");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=3570.63,7351.62");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=4464.12,7347.58");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=1121.98,7056.85");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=1495.39,7123.38");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=2117.63,7224.21");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=2732.16,7091.35");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=3240.79,7189.74");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=4121.85,7034.59");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=4970.98,7228.93");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=5442.37,7050.41");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=506.18,6764.38");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=2225.61,6829.92");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=3467.64,6864.84");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=4624.56,7009.26");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=5143.50,6798.87");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=1197.63,6595.58");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=2553.77,6684.68");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=3673.89,6552.05");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=4813.77,6660.37");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=1782.46,6506.40");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=2355.77,6361.07");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=3100.52,6490.47");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=4438.72,6457.02");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=5686.12,6495.35");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=274.18,6197.47");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=2626.02,6124.61");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=3359.33,6142.14");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=3876.91,6166.44");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=5039.80,6234.95");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=5652.39,6107.69");
    }

}

