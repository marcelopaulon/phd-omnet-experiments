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

network DADCA_ACK_n50_u3_a8000m2_8
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
        @display("i=misc/sensor;p=882.96,7792.43");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1721.49,7764.74");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2323.78,7962.71");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3046.31,7870.65");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3728.08,7981.37");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=5164.03,7762.57");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5609.47,7896.84");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=5946.75,7759.15");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=455.31,7578.12");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=2205.63,7527.64");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=2575.47,7529.34");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=2928.34,7533.71");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=3336.23,7560.75");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=4047.12,7733.85");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=4469.24,7550.90");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=4901.56,7505.38");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=5465.11,7537.18");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=4034.40,7359.88");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=5879.77,7350.05");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=359.90,7125.19");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=800.94,7104.73");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=1334.39,7101.65");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=1879.05,7199.18");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=2550.03,7107.26");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=2923.83,7048.54");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=3516.49,7139.60");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=5036.99,7162.74");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=3843.32,6980.08");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=4373.96,6974.02");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=5655.96,6770.03");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=1104.90,6564.76");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=1635.22,6734.97");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=2305.18,6756.99");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=2685.37,6529.08");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=3330.17,6627.21");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=4947.21,6662.26");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=1733.72,6389.52");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=3689.25,6501.32");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=5244.19,6436.94");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=5937.56,6355.42");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=1014.88,6145.11");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=1423.63,6085.48");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=2143.66,6149.85");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=2819.78,6151.22");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=3233.38,6262.12");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=4260.05,6196.29");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=4788.56,6193.66");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=5605.71,6179.53");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=499.94,5974.53");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=1810.21,5814.62");
    }

}

