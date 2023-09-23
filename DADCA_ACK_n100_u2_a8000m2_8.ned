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

network DADCA_ACK_n100_u2_a8000m2_8
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
        @display("i=misc/sensor;p=583.95,7923.94");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1224.13,7922.00");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1627.66,7793.80");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=4488.04,7848.44");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=5182.75,7950.61");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=5713.71,7930.97");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=899.57,7692.50");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=2122.56,7601.55");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=3080.59,7575.82");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=3488.56,7680.09");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=4885.71,7547.46");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=5361.94,7558.26");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=5997.62,7715.32");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=301.69,7494.63");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1636.79,7436.92");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=3938.18,7474.03");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=578.17,7105.00");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=1241.39,7174.92");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=2152.98,7239.82");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=2566.20,7170.80");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=3349.66,7235.62");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=4444.83,7198.45");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=5123.67,7242.76");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=273.39,6918.48");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=941.64,6953.13");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=1684.86,6967.36");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=2524.39,6806.77");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=4054.32,6959.92");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=4994.45,6881.86");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=5404.22,6959.41");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=1524.35,6611.35");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=2169.52,6532.28");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=3083.29,6591.83");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=3763.40,6674.81");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=4383.36,6701.48");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=5797.57,6680.62");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=198.36,6334.38");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=774.40,6456.84");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=1273.53,6326.96");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=4801.19,6318.53");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=5168.49,6408.59");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=1639.37,6051.44");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=2015.57,6199.53");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=2378.38,6218.85");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=3428.02,6077.88");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=3896.87,6075.37");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=5469.64,6202.17");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=641.35,5886.80");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=1068.13,5980.29");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=2343.11,5828.53");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=4496.01,5901.51");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=5034.71,5874.34");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=5922.71,5875.37");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=63.63,5686.43");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=2610.12,5565.07");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=2990.07,5771.40");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=3572.23,5563.13");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=4057.68,5602.30");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=4754.52,5645.86");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=104.85,5309.82");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=571.39,5301.91");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=1179.68,5471.97");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=1727.67,5380.95");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=2218.56,5452.54");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=5469.39,5427.04");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=1205.18,5064.01");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=1952.13,5104.46");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=2739.98,5168.36");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=3127.67,5210.18");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=3742.88,5077.81");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=4270.70,5244.16");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=4741.07,5241.48");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=5144.36,5213.93");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=5763.35,5073.69");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=0.61,4901.81");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=382.25,4950.75");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=1630.71,4847.70");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=2552.44,4838.81");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=2945.96,4799.89");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=4512.28,4920.71");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=519.75,4560.22");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=1012.97,4705.93");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=3560.36,4762.81");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=4234.44,4674.34");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=5101.22,4724.58");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=5666.81,4692.99");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=1961.52,4531.65");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=2776.32,4318.75");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=3956.85,4454.05");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=4600.17,4520.35");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=5129.09,4321.37");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=15.35,4236.38");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=580.63,4167.23");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=1107.95,4060.17");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=1507.04,4265.07");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=2395.52,4103.55");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=3396.25,4219.82");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=4017.06,4091.12");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=5643.89,4042.80");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=1664.79,3911.52");
    }

}

