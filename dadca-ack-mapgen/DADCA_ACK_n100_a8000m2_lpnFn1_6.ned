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
        @display("i=misc/sensor,p=65.67,16.57");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=131.65,2.71");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=223.85,7.33");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=222.42,82.63");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=194.82,149.41");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=281.19,196.62");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=335.59,271.52");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=340.05,208.35");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=401.08,275.32");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=297.51,82.03");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=357.79,127.55");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=419.32,132.31");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=513.32,98.71");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=400.49,63.09");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=363.03,15.87");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=260.24,258.69");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=250.88,396.70");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=306.48,458.85");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=372.18,472.32");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor,p=346.48,406.50");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor,p=408.15,600.73");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor,p=367.64,535.98");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor,p=295.80,603.49");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor,p=214.84,546.06");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor,p=173.46,619.54");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor,p=212.38,667.89");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor,p=278.55,704.47");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor,p=159.52,412.11");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor,p=100.69,349.05");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor,p=28.75,348.10");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor,p=78.62,429.90");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor,p=37.95,476.43");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor,p=59.90,565.13");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor,p=120.34,543.90");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor,p=148.73,488.42");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor,p=88.83,652.76");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor,p=0.60,599.95");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor,p=132.35,710.16");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor,p=157.92,781.08");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor,p=75.81,731.65");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor,p=356.60,675.57");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor,p=71.07,243.12");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor,p=2.83,209.43");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor,p=37.14,117.93");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor,p=129.35,182.79");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor,p=166.51,283.20");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor,p=201.49,356.53");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor,p=287.10,313.82");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor,p=341.04,345.19");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor,p=450.56,27.38");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor,p=283.31,21.56");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor,p=457.05,184.72");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor,p=568.70,146.62");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor,p=513.52,220.90");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor,p=561.36,51.90");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor,p=635.19,58.09");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor,p=695.37,117.25");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor,p=733.03,198.58");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor,p=706.89,44.56");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor,p=636.70,154.12");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor,p=758.48,93.07");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor,p=784.48,249.05");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor,p=808.15,335.76");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor,p=803.17,396.28");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor,p=866.03,460.59");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor,p=808.40,508.68");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor,p=750.96,467.14");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor,p=676.04,489.95");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor,p=698.90,552.65");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor,p=662.33,604.72");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor,p=603.66,657.61");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor,p=547.34,735.14");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor,p=606.75,789.19");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor,p=535.63,641.77");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor,p=498.03,694.39");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor,p=419.19,682.22");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor,p=451.81,515.17");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor,p=500.12,462.29");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor,p=474.97,582.31");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor,p=30.46,666.97");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor,p=414.61,764.61");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor,p=502.16,786.95");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor,p=478.34,849.79");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor,p=402.19,829.63");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor,p=342.27,870.09");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor,p=235.28,800.31");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor,p=147.82,846.16");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor,p=88.57,833.29");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor,p=40.37,899.25");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor,p=8.62,740.83");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor,p=224.00,461.36");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor,p=20.13,815.85");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor,p=307.10,542.50");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor,p=469.44,275.23");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor,p=404.08,377.47");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor,p=585.49,216.71");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor,p=527.56,377.80");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor,p=555.52,581.82");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor,p=1.68,277.93");
    }

}

