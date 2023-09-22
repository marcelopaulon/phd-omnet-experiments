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

network DADCA_ACK_n50_a8000m2_0
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
        @display("i=misc/sensor;p=484.17,7956.89");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=545.52,7989.68");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=651.35,7976.71");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=802.59,7960.73");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=128.95,7921.60");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=263.31,7917.27");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=375.74,7946.41");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=584.69,7940.31");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=710.89,7950.15");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=906.14,7949.22");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=51.76,7894.66");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=407.65,7883.24");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=588.07,7872.97");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=641.57,7909.35");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=739.25,7894.98");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=802.33,7882.96");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=97.15,7832.10");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=167.15,7864.23");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=232.22,7862.57");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=302.52,7850.70");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=524.45,7849.31");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=687.86,7837.85");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=904.65,7828.32");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=972.10,7865.90");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=30.64,7813.94");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=157.99,7798.39");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=596.58,7802.57");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=254.76,7749.67");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=402.71,7770.27");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=490.78,7751.47");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=714.90,7774.10");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=817.75,7777.96");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=28.07,7740.49");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=342.62,7738.22");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=556.30,7732.98");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=624.47,7717.63");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=775.04,7707.24");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=916.37,7719.53");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=984.94,7739.15");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=176.00,7685.47");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=294.49,7667.35");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=823.99,7661.21");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=60.06,7649.77");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=469.87,7640.14");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=598.73,7646.26");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=695.05,7654.09");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=918.34,7615.38");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=140.04,7609.51");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=209.62,7603.13");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=337.22,7591.18");
    }

}

