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

network DADCA_ACK_n50_u8_a8000m2_0
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
        @display("i=misc/sensor;p=93.63,7988.56");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1244.04,7946.72");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1659.28,7761.74");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2402.90,7932.25");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=2814.64,7903.56");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=3484.57,7935.06");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=4544.63,7773.47");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=5313.62,7940.91");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=5694.55,7940.12");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=708.38,7638.23");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=1203.71,7549.12");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=3176.92,7509.12");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=4123.44,7651.86");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=5007.22,7710.07");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=5553.09,7518.22");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=32.58,7422.78");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=847.12,7264.83");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=2178.49,7269.96");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=2605.07,7481.25");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=3608.32,7342.54");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=4924.15,7366.99");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=5862.00,7328.12");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=508.68,7119.37");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=1657.69,7084.92");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=2842.09,7011.78");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=4021.80,7116.66");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=4500.55,7222.85");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=5227.73,7177.62");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=81.07,6830.00");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=947.58,6763.12");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=1322.98,6945.37");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=3818.28,6769.18");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=4785.36,6990.26");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=5142.74,6792.84");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=5737.53,6873.49");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=1308.78,6544.39");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=2207.36,6737.59");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=2557.72,6721.47");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=3196.32,6719.39");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=4474.86,6588.20");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=5553.63,6556.02");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=405.58,6450.29");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=1959.19,6448.99");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=2368.70,6323.26");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=2908.23,6382.74");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=3401.59,6294.66");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=4003.89,6443.61");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=4924.94,6408.21");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=4147.86,6080.66");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=4825.51,6066.63");
    }

}

