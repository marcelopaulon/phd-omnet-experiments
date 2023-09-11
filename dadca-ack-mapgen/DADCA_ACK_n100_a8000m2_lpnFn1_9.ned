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

network DADCA_ACK_n100_a8000m2_lpnFn1_9
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
        @display("i=misc/sensor,p=38.62,89.34");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=19.20,160.04");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=25.89,256.31");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=33.75,333.26");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=95.39,325.19");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=154.05,247.95");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=238.41,227.54");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=280.09,182.80");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=344.29,157.21");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=338.94,238.71");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=456.43,131.52");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=503.32,65.41");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=595.04,77.31");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=571.24,159.51");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=635.18,201.16");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=403.94,78.14");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=345.64,3.81");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=251.22,5.63");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=263.73,103.55");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor,p=376.49,310.14");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor,p=279.61,360.88");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor,p=442.44,302.76");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor,p=497.11,332.61");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor,p=454.94,202.67");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor,p=535.92,284.24");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor,p=406.53,395.88");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor,p=687.27,142.23");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor,p=662.68,74.55");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor,p=607.32,295.25");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor,p=679.35,264.67");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor,p=569.70,220.17");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor,p=564.09,17.06");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor,p=202.85,102.89");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor,p=322.26,433.92");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor,p=382.79,462.72");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor,p=299.86,503.31");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor,p=355.89,584.21");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor,p=273.15,685.35");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor,p=251.99,757.66");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor,p=414.55,639.16");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor,p=160.16,508.94");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor,p=197.99,440.72");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor,p=78.17,522.38");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor,p=10.67,452.14");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor,p=43.51,574.58");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor,p=103.21,590.06");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor,p=251.64,547.74");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor,p=303.19,852.46");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor,p=267.72,915.00");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor,p=281.48,978.80");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor,p=208.82,968.42");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor,p=147.78,982.05");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor,p=135.94,894.07");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor,p=219.62,834.97");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor,p=375.24,948.37");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor,p=76.00,777.19");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor,p=36.24,847.31");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor,p=46.19,976.27");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor,p=88.32,1019.01");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor,p=33.95,1051.07");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor,p=5.62,1136.74");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor,p=127.65,1090.57");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor,p=86.20,1155.52");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor,p=16.72,1201.18");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor,p=90.12,1224.28");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor,p=53.35,1307.21");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor,p=106.13,1378.26");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor,p=24.91,1372.11");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor,p=92.12,1439.44");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor,p=116.49,1521.38");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor,p=125.04,1616.05");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor,p=172.55,1573.35");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor,p=200.02,1635.97");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor,p=236.76,1698.09");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor,p=310.22,1690.72");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor,p=299.41,1780.99");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor,p=272.47,1596.74");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor,p=277.34,1856.58");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor,p=371.59,1861.31");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor,p=340.55,1931.46");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor,p=392.07,1968.76");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor,p=386.15,2048.71");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor,p=374.63,2110.97");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor,p=287.16,2091.62");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor,p=250.56,1956.90");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor,p=264.88,2035.51");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor,p=431.88,1850.42");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor,p=502.38,1812.28");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor,p=488.53,1883.57");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor,p=526.94,1726.12");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor,p=587.84,1725.29");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor,p=659.51,1753.00");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor,p=637.10,1690.48");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor,p=414.11,1725.27");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor,p=363.53,1639.47");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor,p=458.61,1655.59");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor,p=596.15,1612.37");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor,p=513.69,1613.04");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor,p=540.68,1666.93");
    }

}

