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

network DADCA_ACK_n50_u8_a8000m2_6
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
        @display("i=misc/sensor;p=1315.61,7760.01");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=2256.71,7788.87");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2813.62,7945.22");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3168.64,7786.05");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3863.28,7779.61");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4209.45,7875.21");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=4720.51,7905.15");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=770.54,7731.98");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=1836.95,7657.77");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=2533.85,7540.07");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=4433.03,7598.91");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=5136.73,7632.59");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=5753.62,7602.69");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=1046.14,7368.00");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1404.66,7413.99");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=2914.76,7403.19");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=4951.45,7267.51");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=5322.26,7282.20");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=707.86,7140.09");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=1765.95,7254.35");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=2188.53,7180.00");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=4125.60,7202.76");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=4590.90,7126.16");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=5888.07,7171.51");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=1260.47,6870.37");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=2018.52,6774.56");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=2590.13,6915.96");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=5182.69,6954.44");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=291.84,6677.69");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=1030.13,6548.04");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=3217.28,6752.34");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=3792.78,6660.04");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=4167.42,6741.47");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=4578.02,6647.94");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=5583.82,6633.00");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=575.20,6434.01");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=1419.43,6275.89");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=2126.69,6398.28");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=2734.50,6484.87");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=3322.01,6387.64");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=4448.35,6314.77");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=2428.42,6217.18");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=3548.84,6029.23");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=4154.20,6056.68");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=4930.94,6187.22");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=5961.74,6171.55");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=330.45,5938.10");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=944.29,5924.20");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=2063.85,5819.69");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=2588.98,5896.58");
    }

}

