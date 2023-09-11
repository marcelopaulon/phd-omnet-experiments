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

network DADCA_ACK_n100_a8000m2_lpnFn1_2
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
        @display("i=misc/sensor,p=34.97,152.42");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=161.64,107.92");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=94.29,34.10");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=32.26,55.19");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=67.49,242.08");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=176.94,273.71");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=239.91,206.94");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=316.12,143.70");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=333.27,64.16");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=333.54,2.77");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=427.37,42.95");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=509.89,47.40");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=598.38,71.53");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=579.09,141.02");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=690.65,182.16");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=638.85,260.81");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=650.03,343.17");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=563.57,310.78");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=555.62,403.76");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor,p=635.90,451.57");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor,p=571.11,507.94");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor,p=714.53,363.99");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor,p=526.31,561.97");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor,p=502.51,621.44");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor,p=596.00,588.50");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor,p=580.05,681.37");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor,p=495.54,721.93");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor,p=453.45,661.86");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor,p=410.55,810.71");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor,p=423.91,897.48");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor,p=522.04,879.93");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor,p=616.51,960.23");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor,p=632.97,875.69");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor,p=701.72,893.06");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor,p=680.34,962.15");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor,p=636.24,1098.69");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor,p=580.19,1154.85");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor,p=539.64,1106.50");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor,p=462.89,1124.69");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor,p=405.68,1149.52");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor,p=404.42,1234.61");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor,p=413.22,1308.34");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor,p=409.26,1407.18");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor,p=443.41,1457.92");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor,p=418.08,1535.44");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor,p=496.62,1496.52");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor,p=486.35,1564.83");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor,p=547.79,1586.98");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor,p=596.22,1546.13");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor,p=652.89,1593.31");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor,p=719.02,1626.09");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor,p=670.53,1667.84");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor,p=725.10,1708.57");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor,p=798.62,1666.24");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor,p=605.60,1652.27");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor,p=528.16,1667.05");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor,p=454.46,1657.26");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor,p=767.54,1589.21");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor,p=851.10,1589.82");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor,p=566.54,1728.83");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor,p=506.28,1770.38");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor,p=575.22,1800.83");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor,p=471.93,1849.87");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor,p=404.73,1768.33");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor,p=361.44,1721.01");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor,p=558.76,1489.47");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor,p=456.20,1914.94");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor,p=496.09,1959.88");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor,p=440.17,2014.29");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor,p=439.99,2088.73");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor,p=379.01,2013.91");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor,p=284.44,1985.04");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor,p=303.42,1926.00");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor,p=266.57,1855.86");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor,p=341.07,1808.48");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor,p=202.88,1928.62");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor,p=140.24,1985.55");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor,p=113.97,2065.99");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor,p=220.40,2051.16");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor,p=15.29,2058.54");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor,p=63.30,1998.84");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor,p=52.22,1899.98");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor,p=30.65,1815.48");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor,p=122.14,1855.83");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor,p=66.70,1703.37");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor,p=47.68,1644.96");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor,p=122.14,1748.61");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor,p=166.47,1681.20");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor,p=105.61,1617.28");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor,p=172.05,1610.44");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor,p=264.74,1649.45");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor,p=115.81,1557.19");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor,p=343.75,1622.53");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor,p=557.04,1870.12");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor,p=660.41,1936.48");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor,p=671.19,2011.69");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor,p=604.91,2031.96");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor,p=404.36,1849.30");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor,p=494.19,2154.01");
    }

}

