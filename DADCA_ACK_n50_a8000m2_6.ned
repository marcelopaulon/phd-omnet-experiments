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

network DADCA_ACK_n50_a8000m2_6
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
        @display("i=misc/sensor;p=10.86,7791.40");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=336.78,7947.07");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1001.79,7963.99");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=1495.57,7857.40");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=1906.45,7845.97");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=2258.30,7856.09");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=2808.78,7920.43");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=3963.59,7842.68");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=4997.47,7932.42");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=5544.50,7848.08");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=564.83,7629.44");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=2653.77,7599.54");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=3478.25,7605.33");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=4558.79,7584.56");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=5829.01,7578.90");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=133.38,7337.39");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=912.44,7413.65");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=1424.72,7280.95");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=1937.53,7494.85");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=2417.90,7313.00");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=3188.44,7326.12");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=5170.38,7451.39");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=5533.20,7346.36");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=521.71,7136.68");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=2797.90,7217.02");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=3534.50,7036.23");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=3912.14,7148.23");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=4796.79,7168.85");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=37.39,6928.82");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=835.08,6977.01");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=1244.61,6917.87");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=2038.93,6943.49");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=2620.08,6897.97");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=3172.28,6901.34");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=5154.50,6982.05");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=5662.18,6844.42");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=272.41,6591.67");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=743.99,6591.75");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=1666.05,6721.54");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=3356.38,6585.72");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=3802.05,6525.67");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=4295.55,6694.43");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=5087.79,6575.80");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=1254.66,6512.45");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=1801.61,6345.48");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=2193.00,6481.28");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=2724.58,6385.89");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=4641.61,6280.11");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=76.30,6101.86");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=416.42,6222.39");
    }

}

