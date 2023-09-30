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

network DADCA_ACK_n50_u4_a8000m2_6
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
        @display("i=misc/sensor;p=1610.67,7837.86");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=4085.36,7891.35");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=4438.92,7883.40");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=4931.05,7992.58");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=5749.99,7989.55");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=494.31,7550.52");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=958.58,7692.17");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=2415.08,7701.55");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=3350.96,7673.25");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=4797.92,7537.22");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=5470.88,7512.87");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=5985.89,7666.08");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=0.70,7281.30");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=2980.53,7490.73");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=3777.70,7431.55");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=4361.78,7306.78");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=5104.32,7342.30");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=1535.78,7246.44");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=1918.98,7249.16");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=2320.04,7109.69");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=2673.34,7184.79");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=326.89,6916.74");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=728.79,6940.81");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=1065.10,6843.88");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=1741.59,6892.62");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=2728.65,6820.39");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=3075.95,7008.52");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=3982.02,6910.18");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=4705.10,6799.29");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=5568.73,6932.94");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=110.70,6599.79");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=2206.88,6754.48");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=4319.63,6549.60");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=5272.42,6540.77");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=5926.28,6609.99");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=350.75,6306.39");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=755.36,6503.89");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=1414.67,6351.28");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=1881.12,6299.03");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=2324.27,6300.71");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=2738.87,6346.25");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=3382.27,6477.96");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=3681.86,6267.88");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=3081.60,6233.18");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=4075.33,6213.30");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=4917.13,6190.30");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=5498.68,6063.94");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=9.57,5870.54");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=490.77,5841.07");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=1205.59,5943.40");
    }

}

