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

network DADCA_ACK_n50_u2_a8000m2_2
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
        @display("i=misc/sensor;p=311.40,7972.30");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1979.43,7796.36");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2423.90,7998.74");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3163.41,7824.08");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4059.82,7933.50");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4535.75,7824.07");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5526.98,7986.64");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=586.33,7746.32");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=1161.35,7612.46");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=2766.38,7661.33");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=5025.13,7693.03");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=5410.95,7645.44");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=1560.36,7434.10");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=2062.23,7396.67");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=2514.56,7355.30");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=3063.00,7465.61");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=3413.39,7468.18");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=3768.60,7499.07");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=4454.54,7462.06");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=4964.05,7321.73");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=857.29,7190.44");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=2198.79,7065.65");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=3156.17,7107.67");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=3535.55,7109.91");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=3887.59,7052.84");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=4548.84,7019.38");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=5356.03,7241.60");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=39.51,6911.33");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=1267.23,6945.54");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=1832.37,6866.68");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=2997.37,6790.56");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=4249.87,6806.36");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=777.44,6704.04");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=1381.44,6603.47");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=2347.43,6717.50");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=3230.36,6523.48");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=3673.35,6654.87");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=4673.71,6648.49");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=5368.31,6659.00");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=5969.58,6708.32");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=1058.25,6377.28");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=1885.86,6390.00");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=3807.08,6315.03");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=4308.64,6282.15");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=5905.08,6297.82");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=139.26,6258.63");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=638.06,6196.25");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=1481.08,6174.00");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=2173.14,6137.55");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=2852.80,6142.76");
    }

}

