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

network DADCA_ACK_n50_u4_a8000m2_5
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
        @display("i=misc/sensor;p=1468.67,7802.71");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=2713.29,7828.26");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=3252.00,7985.32");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3944.77,7904.29");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4612.07,7901.95");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=5195.91,7929.85");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=2071.37,7553.87");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=3425.34,7630.42");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=4360.55,7533.85");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=4918.84,7530.73");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=5365.94,7613.28");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=5822.21,7731.87");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=1001.62,7418.31");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=1599.73,7362.52");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=2949.75,7328.68");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=3931.09,7284.92");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=5469.84,7268.55");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=873.74,7011.21");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=2473.78,7121.63");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=3465.88,7187.25");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=4466.12,7077.07");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=529.84,6854.13");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=1558.89,6992.34");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=2034.54,6924.35");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=2774.46,6785.16");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=3177.45,6827.23");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=4045.32,6938.31");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=4729.58,6840.98");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=5940.28,6957.18");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=71.30,6602.50");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=704.95,6523.52");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=1310.12,6609.35");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=2356.54,6719.51");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=5076.66,6616.22");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=5454.53,6642.81");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=5848.74,6516.79");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=403.09,6298.98");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=1512.74,6320.63");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=2173.02,6302.95");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=2600.77,6333.31");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=2984.84,6481.37");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=3778.57,6513.58");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=4438.33,6477.99");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=1844.76,6045.14");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=2923.29,6056.68");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=4143.18,6085.38");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=4671.89,6066.18");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=4990.68,6261.64");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=5333.68,6048.96");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=5944.46,6047.74");
    }

}

