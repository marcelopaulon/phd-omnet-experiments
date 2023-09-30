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

network DADCA_ACK_n50_u8_a8000m2_3
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
        @display("i=misc/sensor;p=1604.95,7785.98");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=2289.69,7917.43");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2753.57,7926.28");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3434.63,7963.19");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3969.20,7946.64");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4993.48,7963.25");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5577.73,7920.96");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=8.64,7589.84");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=407.02,7706.79");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=1974.55,7509.40");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=2389.32,7577.64");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=3070.11,7730.37");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=4085.52,7535.27");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=4620.80,7739.36");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1240.43,7317.86");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=3266.12,7369.05");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=3705.25,7460.04");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=5056.94,7447.36");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=5746.52,7457.52");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=586.40,7242.30");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=1520.49,7017.57");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=2823.25,7130.82");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=3838.39,7135.07");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=4310.79,7240.99");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=4647.38,7137.07");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=403.47,6861.43");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=996.66,6951.75");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=2007.26,6854.79");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=3046.77,6805.69");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=4833.42,6766.01");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=5310.28,6868.41");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=843.15,6609.32");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=1538.80,6643.20");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=2567.54,6705.61");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=3690.30,6558.69");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=4369.19,6638.16");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=36.50,6315.84");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=574.48,6348.92");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=1174.34,6461.01");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=1763.98,6275.28");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=2577.14,6329.46");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=3176.72,6444.84");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=5359.61,6288.70");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=5887.48,6426.44");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=1323.95,6045.39");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=2166.93,6171.03");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=2952.69,6050.26");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=4189.47,6195.41");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=4879.69,6210.19");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=5945.84,6038.45");
    }

}

