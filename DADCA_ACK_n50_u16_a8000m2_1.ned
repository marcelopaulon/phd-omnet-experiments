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

network DADCA_ACK_n50_u16_a8000m2_1
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
        @display("i=misc/sensor;p=2546.27,7775.73");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=2997.04,7912.14");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=3523.54,7964.37");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=1134.33,7546.55");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=2053.92,7669.14");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4186.91,7584.93");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=4782.88,7666.72");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=5220.36,7706.81");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=5649.67,7684.80");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=575.54,7495.26");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=1626.33,7472.75");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=2084.87,7279.71");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=2703.88,7413.84");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=3784.60,7330.53");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=5417.01,7286.22");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=5969.41,7408.38");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=398.18,7160.63");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=861.23,7062.99");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=1262.01,7022.25");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=3127.20,7067.61");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=4143.70,7096.72");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=4524.04,7101.47");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=5063.07,7144.12");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=595.14,6816.93");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=1749.46,6958.04");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=2422.56,6992.96");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=5358.98,6811.98");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=104.79,6734.72");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=988.98,6569.76");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=2068.59,6714.35");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=2734.91,6623.54");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=3121.07,6576.73");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=3521.45,6725.52");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=4146.89,6597.62");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=4563.59,6658.91");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=5029.30,6560.97");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=44.92,6281.61");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=600.39,6332.16");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=1511.29,6482.68");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=1843.00,6327.69");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=2629.56,6277.42");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=5415.93,6436.28");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=5784.79,6418.65");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=941.28,6213.19");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=1588.08,6023.84");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=2276.60,6184.51");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=3770.40,6033.41");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=4265.45,6024.45");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=4615.90,6207.96");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=2930.51,5966.71");
    }

}

