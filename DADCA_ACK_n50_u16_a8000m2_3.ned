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

network DADCA_ACK_n50_u16_a8000m2_3
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
        @display("i=misc/sensor;p=655.36,7867.25");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1202.36,7825.45");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1913.63,7974.52");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2509.70,7767.70");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3006.98,7813.24");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=5273.89,7880.60");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=208.39,7562.03");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=3675.88,7508.98");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=4228.71,7713.84");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=4597.11,7734.44");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=5598.76,7542.38");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=575.76,7320.28");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=1031.24,7319.54");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=1656.19,7401.77");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=2286.92,7417.89");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=2914.00,7425.09");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=3282.48,7261.75");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=4113.41,7318.67");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=4735.91,7290.46");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=5120.99,7417.97");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=215.65,7100.93");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=3756.54,7036.67");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=5969.00,7075.89");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=707.69,6814.92");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=1298.13,6997.97");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=2394.26,6943.76");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=2816.36,6958.56");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=3324.13,6906.69");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=4428.78,6843.88");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=4909.45,6972.79");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=5482.31,6911.25");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=329.29,6703.38");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=1063.99,6735.32");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=2063.28,6752.20");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=3522.90,6524.69");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=4651.42,6548.87");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=5163.23,6612.70");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=755.06,6402.77");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=1150.85,6327.81");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=1638.99,6415.99");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=2412.86,6443.82");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=3068.01,6278.86");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=4208.98,6482.21");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=189.02,6067.02");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=1680.06,6020.90");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=2122.63,6246.99");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=2663.22,6190.13");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=3693.60,6143.81");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=5714.57,6225.61");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=565.14,5831.64");
    }

}

