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

network DADCA_ACK_n50_u4_a8000m2_1
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
        @display("i=misc/sensor;p=253.66,7889.78");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=928.12,7967.76");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1790.01,7940.25");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2277.71,7958.37");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=2805.93,7822.05");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=3160.63,7948.85");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=3650.49,7821.72");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=5736.26,7984.82");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=709.95,7625.43");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=1374.93,7625.60");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=2052.91,7556.03");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=3066.74,7551.38");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=4309.32,7711.67");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=4736.88,7583.27");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=5238.25,7728.62");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=5587.70,7588.36");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=386.20,7406.68");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=1118.85,7372.71");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=2484.07,7316.87");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=3544.35,7450.38");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=699.26,7170.37");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=2145.14,7135.91");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=3548.36,7047.40");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=3914.30,7216.34");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=4529.90,7140.29");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=5248.13,7236.68");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=5652.19,7161.16");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=573.29,6797.38");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=1014.51,6971.11");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=1523.30,6990.66");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=2935.86,6996.06");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=5041.09,6846.83");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=5412.99,6806.76");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=5981.99,6790.43");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=941.00,6519.86");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=1282.64,6609.62");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=1853.24,6745.64");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=2576.14,6611.96");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=3542.41,6637.77");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=3941.99,6751.20");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=4468.93,6626.39");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=1623.05,6449.46");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=2233.24,6460.59");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=3018.89,6304.10");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=4866.87,6448.98");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=5303.85,6431.99");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=5743.84,6319.62");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=152.03,6208.59");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=676.21,6145.15");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=1208.86,6189.80");
    }

}

