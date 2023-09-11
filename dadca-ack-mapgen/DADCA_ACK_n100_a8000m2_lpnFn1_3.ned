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

network DADCA_ACK_n100_a8000m2_lpnFn1_3
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
        @display("i=misc/sensor,p=87.13,37.34");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=157.35,70.03");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=65.56,149.15");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=244.88,106.48");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=212.90,12.40");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=23.14,83.63");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=16.25,11.68");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=197.83,177.03");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=268.64,247.49");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=338.51,284.96");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=391.43,255.73");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=453.38,323.28");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=461.53,250.44");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=495.03,168.48");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=533.06,220.26");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=466.78,99.33");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=484.73,34.58");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=361.44,176.93");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=406.26,121.36");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor,p=425.19,196.69");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor,p=374.56,343.25");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor,p=373.68,55.91");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor,p=311.36,82.95");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor,p=271.40,177.12");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor,p=200.10,267.99");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor,p=646.79,239.74");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor,p=567.80,298.42");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor,p=480.59,413.68");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor,p=534.42,387.09");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor,p=399.91,435.87");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor,p=451.47,513.94");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor,p=377.37,502.79");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor,p=313.92,426.48");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor,p=240.73,514.22");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor,p=312.82,498.33");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor,p=178.72,457.26");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor,p=190.78,389.80");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor,p=123.27,421.95");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor,p=56.61,390.69");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor,p=161.53,535.21");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor,p=288.98,360.00");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor,p=571.21,459.78");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor,p=513.64,496.76");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor,p=582.53,557.43");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor,p=558.71,614.15");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor,p=464.98,609.11");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor,p=408.42,581.93");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor,p=557.27,10.06");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor,p=528.58,107.47");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor,p=141.90,286.11");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor,p=86.29,240.74");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor,p=29.84,296.36");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor,p=4.53,238.63");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor,p=140.69,210.88");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor,p=240.40,318.49");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor,p=89.99,316.84");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor,p=81.76,470.21");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor,p=99.61,533.96");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor,p=250.73,430.14");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor,p=234.97,613.52");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor,p=81.10,611.91");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor,p=94.12,680.34");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor,p=154.79,626.15");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor,p=20.07,491.53");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor,p=626.82,416.32");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor,p=692.66,487.20");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor,p=747.71,423.20");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor,p=836.58,521.64");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor,p=911.54,498.19");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor,p=964.98,568.26");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor,p=928.50,641.38");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor,p=904.79,716.12");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor,p=997.28,686.84");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor,p=989.75,783.16");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor,p=1081.82,698.25");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor,p=1077.54,792.38");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor,p=1038.29,882.61");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor,p=1114.29,937.68");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor,p=1190.89,957.44");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor,p=1192.72,1035.02");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor,p=1136.68,1089.17");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor,p=1043.73,1114.76");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor,p=1034.34,1199.84");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor,p=980.30,1227.23");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor,p=925.60,1274.18");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor,p=906.92,1176.71");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor,p=870.56,1120.16");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor,p=790.86,1159.89");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor,p=727.44,1135.22");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor,p=840.90,1060.34");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor,p=984.46,1084.23");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor,p=929.86,1046.83");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor,p=983.87,1007.21");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor,p=1074.10,987.49");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor,p=1004.91,936.01");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor,p=967.47,872.61");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor,p=907.71,815.21");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor,p=915.20,902.70");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor,p=848.17,912.30");
    }

}

