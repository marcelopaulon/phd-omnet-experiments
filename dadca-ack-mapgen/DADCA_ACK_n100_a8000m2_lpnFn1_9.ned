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
        @display("i=misc/sensor,p=41.45,180.74");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=9.69,114.88");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=113.76,296.86");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=151.67,346.82");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=105.25,224.83");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=222.05,274.59");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=176.55,212.56");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=216.04,161.20");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=177.55,111.16");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=163.72,46.74");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=245.92,80.68");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=288.65,129.95");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=335.40,74.06");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=356.65,173.16");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=429.46,112.55");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=499.98,112.75");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=432.22,15.33");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=299.42,236.80");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=283.05,311.35");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor,p=312.03,406.76");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor,p=240.49,355.39");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor,p=43.72,293.84");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor,p=82.93,35.47");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor,p=461.36,203.93");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor,p=455.36,264.01");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor,p=515.91,329.71");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor,p=543.22,268.06");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor,p=473.02,415.46");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor,p=395.81,355.44");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor,p=276.45,457.32");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor,p=378.47,419.37");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor,p=430.15,476.73");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor,p=408.87,534.70");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor,p=454.41,610.43");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor,p=356.04,609.99");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor,p=339.13,685.10");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor,p=352.06,764.02");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor,p=343.55,851.08");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor,p=286.72,877.64");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor,p=342.14,933.06");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor,p=423.40,956.43");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor,p=364.60,993.19");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor,p=429.53,896.10");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor,p=278.40,1079.23");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor,p=369.98,1075.61");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor,p=445.62,1045.54");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor,p=474.15,990.77");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor,p=283.29,1018.38");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor,p=163.93,1008.20");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor,p=215.94,932.48");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor,p=217.12,869.51");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor,p=87.65,830.61");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor,p=26.58,799.35");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor,p=8.18,708.34");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor,p=110.42,733.02");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor,p=1.05,635.46");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor,p=3.65,550.48");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor,p=71.66,547.20");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor,p=65.94,615.69");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor,p=65.64,679.13");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor,p=162.66,654.30");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor,p=131.39,599.19");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor,p=153.51,509.10");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor,p=255.62,568.40");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor,p=183.98,569.99");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor,p=36.64,481.44");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor,p=255.29,655.07");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor,p=487.07,746.35");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor,p=423.37,707.70");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor,p=349.30,513.88");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor,p=293.26,751.44");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor,p=486.77,542.73");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor,p=575.68,648.09");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor,p=508.22,685.56");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor,p=591.54,723.03");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor,p=558.10,821.01");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor,p=627.56,839.02");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor,p=592.36,905.31");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor,p=672.56,931.74");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor,p=768.80,874.78");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor,p=735.00,950.89");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor,p=705.62,1013.16");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor,p=677.09,1109.47");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor,p=742.17,1173.86");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor,p=826.50,1158.23");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor,p=908.79,1185.27");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor,p=974.77,1170.49");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor,p=839.23,1292.29");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor,p=930.18,1296.26");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor,p=1022.18,1292.64");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor,p=1023.40,1213.84");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor,p=995.84,1063.06");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor,p=1037.92,1124.16");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor,p=1058.80,1058.46");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor,p=1121.32,1109.39");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor,p=1106.97,1200.45");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor,p=1152.12,1274.68");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor,p=1218.86,1229.64");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor,p=1205.62,1169.28");
    }

}

