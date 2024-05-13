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

network DADCA_ACK_n50_u4_a8000m2_3
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
        @display("i=misc/sensor;p=1846.53,7789.37");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=3306.97,7813.99");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=3995.90,7907.81");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=4355.73,7759.85");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=5757.14,7943.70");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=47.20,7733.91");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=532.42,7636.77");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=2801.24,7570.98");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=5218.68,7560.76");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=5950.47,7650.72");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=73.39,7348.82");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=1033.35,7456.67");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=2053.89,7278.42");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=2367.36,7464.04");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=3648.50,7429.51");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=4594.43,7383.87");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=5872.62,7297.34");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=447.47,7240.52");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=852.54,7027.65");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=1537.13,7252.68");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=2681.20,7019.40");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=3194.95,7157.47");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=4151.45,7098.88");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=5147.51,7011.89");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=5548.37,7126.85");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=314.90,6812.95");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=1213.54,6852.29");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=1899.58,6954.72");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=3698.40,6797.90");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=4348.56,6798.45");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=4779.86,6906.18");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=5861.94,6937.74");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=1494.09,6628.42");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=1837.78,6515.97");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=2186.67,6684.93");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=3129.51,6611.94");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=5318.92,6655.01");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=5770.78,6566.80");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=691.12,6322.34");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=1021.64,6444.15");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=2556.58,6336.31");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=4591.22,6499.57");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=5000.08,6283.61");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=337.44,6076.26");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=1301.70,6106.49");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=2089.17,6254.58");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=3062.86,6117.00");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=3406.19,6227.15");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=4097.72,6159.25");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=1101.67,5798.80");
    }

}

