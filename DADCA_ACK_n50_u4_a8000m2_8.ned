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

network DADCA_ACK_n50_u4_a8000m2_8
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
        @display("i=misc/sensor;p=3571.42,7950.22");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=4031.14,7907.76");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=899.34,7555.20");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=1571.78,7572.07");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=2235.43,7734.15");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=2617.90,7718.88");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=3156.63,7540.75");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=3739.60,7561.32");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=5144.55,7720.16");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=5617.49,7577.37");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=305.06,7475.05");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=2244.83,7334.19");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=4042.43,7289.87");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=126.71,7042.21");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=855.32,7101.54");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=1284.32,7027.66");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=1869.39,7101.27");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=2778.16,7186.03");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=3261.29,7090.88");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=4818.62,7033.46");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=5342.63,7092.58");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=471.54,6974.53");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=2408.94,6815.57");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=3679.31,6852.75");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=4440.42,6972.29");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=104.31,6645.02");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=602.22,6608.51");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=982.11,6690.76");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=1680.08,6639.68");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=2786.77,6744.19");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=5207.78,6678.67");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=5669.47,6588.65");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=1210.28,6389.71");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=2156.98,6413.30");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=3130.58,6445.35");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=4246.89,6397.50");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=4606.46,6345.17");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=306.40,6178.97");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=718.34,6191.68");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=1910.45,6139.41");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=2731.06,6152.05");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=3744.71,6162.15");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=5086.15,6126.69");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=1201.68,5916.68");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=1557.80,5981.60");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=2045.82,5805.06");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=3117.68,5927.82");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=4121.04,5788.72");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=4478.74,6008.78");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=5670.94,5991.77");
    }

}

