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

network DADCA_ACK_n50_u2_a8000m2_6
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
        @display("i=misc/sensor;p=89.94,7758.29");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=591.41,7907.68");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1239.77,7937.85");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2428.02,7971.12");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4314.51,7909.67");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4722.20,7846.94");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5167.70,7940.70");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=5658.95,7838.93");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=730.61,7528.10");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=1269.63,7513.52");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=1761.60,7729.45");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=2218.22,7551.90");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=2847.42,7663.55");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=3422.49,7724.57");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=3905.14,7702.02");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=5306.45,7519.63");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=373.62,7292.23");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=1989.24,7285.16");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=4268.63,7362.60");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=4832.70,7325.60");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=854.29,7192.95");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=1292.43,7118.83");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=1651.79,7149.76");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=2579.83,7119.14");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=3239.78,7228.09");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=5158.90,7038.39");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=5719.58,7094.22");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=258.66,6914.70");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=2120.20,6913.96");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=2759.95,6763.42");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=4002.84,6906.93");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=4664.33,6791.03");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=788.75,6593.72");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=1200.34,6649.48");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=1554.89,6725.11");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=2238.71,6551.04");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=3174.80,6627.98");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=3996.79,6553.41");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=5838.41,6631.50");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=428.50,6512.14");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=1799.51,6390.70");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=5056.96,6348.38");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=5422.49,6463.25");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=314.91,6114.61");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=748.59,6209.01");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=1107.54,6047.45");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=2249.30,6167.14");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=2819.22,6078.97");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=3350.11,6231.56");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=3787.45,6135.36");
    }

}

