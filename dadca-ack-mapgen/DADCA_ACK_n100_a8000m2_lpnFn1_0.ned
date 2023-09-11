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

network DADCA_ACK_n100_a8000m2_lpnFn1_0
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
        @display("i=misc/sensor,p=72.49,25.92");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=39.42,112.09");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=26.27,196.55");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=95.96,246.66");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=157.27,178.48");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=156.76,263.25");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=91.12,176.71");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=44.86,297.50");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=36.10,382.77");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=99.75,405.75");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=194.90,380.07");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=195.80,457.53");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=255.15,529.29");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=224.34,614.22");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=323.44,622.36");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=404.79,546.01");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=446.58,480.21");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=516.87,430.61");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=512.66,354.98");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor,p=589.75,321.01");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor,p=545.83,248.09");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor,p=669.99,343.53");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor,p=690.33,438.87");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor,p=719.20,497.77");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor,p=597.61,465.12");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor,p=607.43,532.44");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor,p=818.19,508.65");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor,p=876.00,473.26");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor,p=869.75,568.77");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor,p=883.39,642.07");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor,p=827.87,617.21");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor,p=732.15,604.34");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor,p=766.23,400.24");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor,p=830.66,347.68");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor,p=823.88,269.93");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor,p=837.55,194.06");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor,p=894.04,303.51");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor,p=765.46,165.54");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor,p=696.08,182.46");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor,p=669.31,258.66");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor,p=748.54,297.73");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor,p=749.11,225.04");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor,p=617.34,190.15");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor,p=597.99,101.46");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor,p=941.95,184.56");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor,p=943.92,86.86");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor,p=880.45,88.89");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor,p=800.79,101.70");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor,p=722.81,30.17");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor,p=930.08,21.91");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor,p=866.63,5.66");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor,p=715.49,125.19");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor,p=932.93,252.03");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor,p=991.66,306.29");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor,p=1086.74,320.02");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor,p=1074.65,412.13");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor,p=1009.57,405.77");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor,p=872.60,411.34");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor,p=461.99,300.62");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor,p=411.71,384.34");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor,p=532.37,526.30");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor,p=636.39,621.00");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor,p=685.00,690.08");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor,p=551.55,642.70");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor,p=559.21,726.01");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor,p=462.66,595.94");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor,p=453.87,675.90");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor,p=378.60,656.23");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor,p=333.30,720.55");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor,p=257.39,691.54");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor,p=250.19,789.97");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor,p=172.23,672.66");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor,p=132.27,729.45");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor,p=185.97,763.49");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor,p=132.74,795.76");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor,p=192.50,887.51");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor,p=302.82,820.78");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor,p=266.72,912.13");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor,p=339.91,943.44");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor,p=402.57,895.61");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor,p=499.79,884.66");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor,p=367.69,839.98");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor,p=94.11,850.07");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor,p=80.68,921.80");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor,p=136.73,947.14");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor,p=22.26,883.61");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor,p=218.13,949.11");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor,p=9.86,760.27");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor,p=57.36,655.33");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor,p=110.99,604.70");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor,p=36.13,540.18");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor,p=72.32,475.13");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor,p=125.01,506.04");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor,p=163.85,567.66");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor,p=272.58,439.61");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor,p=258.31,379.11");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor,p=225.23,254.96");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor,p=258.69,200.14");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor,p=338.11,118.04");
    }

}

