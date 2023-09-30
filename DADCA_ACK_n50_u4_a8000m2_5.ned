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
        @display("i=misc/sensor;p=47.02,7810.79");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=661.28,7791.78");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1434.38,7929.09");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2173.59,7812.12");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3164.14,7990.65");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4765.03,7827.62");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=1161.52,7644.54");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=3183.23,7534.77");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=3516.22,7662.85");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=5147.54,7656.80");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=5821.40,7649.32");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=276.83,7473.61");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=815.95,7443.37");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=1714.87,7329.16");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=2740.18,7410.39");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=3759.97,7375.00");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=4098.77,7494.95");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=4420.20,7293.19");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=5495.02,7303.47");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=1490.57,7012.86");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=2421.84,7243.16");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=3323.54,7198.48");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=4883.31,7242.42");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=198.70,6982.55");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=585.96,6958.05");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=1947.34,6819.17");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=2706.34,7005.82");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=389.98,6550.94");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=903.89,6644.45");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=2389.87,6621.17");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=3036.68,6726.66");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=3591.08,6728.36");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=4051.73,6758.72");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=4813.35,6740.12");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=5353.75,6606.63");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=1220.45,6360.87");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=1768.88,6394.34");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=2645.47,6350.64");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=3284.79,6269.58");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=4357.31,6356.48");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=4802.55,6314.20");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=5755.98,6483.47");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=618.98,6024.14");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=3927.13,6202.26");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=5152.58,6265.98");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=5613.68,6044.09");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=1283.21,5962.10");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=1748.25,5788.45");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=2774.97,5981.56");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=3549.82,6017.18");
    }

}

