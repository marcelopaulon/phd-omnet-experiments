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

network DADCA_ACK_n100_a8000m2_lpnFn1_6
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
        @display("i=misc/sensor,p=100.00,100.00");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor,p=73.76,168.58");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=15.04,211.42");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=134.22,283.75");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=212.86,268.99");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=286.20,313.63");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=226.89,335.33");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=311.30,368.84");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=376.89,363.63");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=324.75,469.65");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=256.78,512.34");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=359.93,551.66");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=333.27,613.46");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=398.24,616.09");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=453.76,691.54");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=438.46,768.06");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=503.85,774.43");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=571.97,730.12");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=670.36,741.01");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=632.21,802.83");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor,p=610.94,894.99");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor,p=525.68,849.91");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor,p=365.23,837.81");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor,p=427.89,830.06");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor,p=502.48,644.12");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor,p=533.67,507.18");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor,p=625.77,515.15");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor,p=574.51,432.34");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor,p=668.72,584.86");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor,p=646.33,662.75");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor,p=376.21,719.62");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor,p=280.38,720.79");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor,p=272.46,625.65");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor,p=398.22,437.30");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor,p=471.87,410.76");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor,p=622.03,313.75");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor,p=636.05,251.38");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor,p=557.78,200.82");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor,p=580.00,139.05");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor,p=660.65,161.11");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor,p=649.07,371.53");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor,p=545.52,312.03");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor,p=460.72,292.91");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor,p=415.86,243.38");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor,p=360.22,202.00");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor,p=418.83,144.35");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor,p=351.26,125.99");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor,p=266.52,167.97");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor,p=210.42,190.98");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor,p=195.86,80.72");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor,p=140.24,4.70");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor,p=79.82,3.14");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor,p=40.54,73.22");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor,p=215.42,398.71");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor,p=450.33,354.61");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor,p=464.39,471.63");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor,p=601.04,593.62");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor,p=495.12,562.48");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor,p=434.38,535.30");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor,p=268.58,435.82");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor,p=171.84,514.18");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor,p=216.85,561.20");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor,p=172.75,671.90");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor,p=222.36,751.29");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor,p=75.27,547.16");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor,p=161.43,611.33");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor,p=99.65,481.59");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor,p=88.57,415.29");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor,p=61.70,355.96");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor,p=9.93,404.78");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor,p=373.79,296.32");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor,p=82.78,628.26");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor,p=27.38,590.95");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor,p=74.59,752.94");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor,p=143.42,790.91");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor,p=216.59,827.41");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor,p=235.51,891.62");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor,p=303.61,851.86");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor,p=431.96,891.33");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor,p=488.48,964.24");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor,p=578.38,1014.89");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor,p=718.23,520.17");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor,p=768.91,609.02");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor,p=794.89,545.26");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor,p=775.74,702.32");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor,p=776.10,447.26");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor,p=848.08,397.27");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor,p=862.79,684.91");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor,p=851.69,588.31");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor,p=872.72,469.10");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor,p=925.18,559.67");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor,p=1008.28,519.16");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor,p=1029.66,435.03");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor,p=1082.55,485.77");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor,p=1074.70,388.30");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor,p=1058.48,590.59");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor,p=967.87,411.43");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor,p=931.97,495.36");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor,p=951.80,649.60");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor,p=995.36,691.06");
    }

}

