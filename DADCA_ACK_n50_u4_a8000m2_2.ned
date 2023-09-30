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

network DADCA_ACK_n50_u4_a8000m2_2
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
        @display("i=misc/sensor;p=705.15,7759.09");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1627.26,7842.19");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=3500.98,7998.54");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=4357.72,7828.83");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=5222.27,7972.84");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=153.19,7693.60");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=1959.42,7608.06");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=2780.90,7690.31");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=3475.81,7566.43");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=3986.67,7725.14");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=5444.59,7676.37");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=5908.85,7623.38");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=951.26,7393.71");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=1639.82,7329.57");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=2460.92,7481.72");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=3893.74,7342.08");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=4804.22,7430.51");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=74.17,7014.26");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=641.21,7090.88");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=2379.54,7137.92");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=2895.28,7183.98");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=4240.77,7131.31");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=5333.39,7048.76");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=5905.21,7011.84");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=1195.14,6993.03");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=2085.41,6878.88");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=2659.21,6766.01");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=3229.56,6952.80");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=4869.40,6844.56");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=271.06,6565.96");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=931.68,6732.11");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=3801.94,6752.40");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=4397.14,6751.39");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=5278.28,6590.90");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=1202.39,6499.81");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=1607.25,6428.06");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=2391.99,6396.66");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=2825.11,6302.49");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=3384.18,6420.78");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=3752.46,6388.16");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=4132.47,6466.61");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=139.10,6021.14");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=709.22,6120.83");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=1155.26,6135.02");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=1809.42,6099.23");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=4701.30,6075.29");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=5596.58,6137.61");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=1061.53,5792.07");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=1480.17,5860.37");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=2307.94,5864.25");
    }

}

