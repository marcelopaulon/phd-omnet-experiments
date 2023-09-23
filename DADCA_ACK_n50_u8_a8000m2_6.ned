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

network DADCA_ACK_n50_u8_a8000m2_6
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
        @display("i=misc/sensor;p=322.77,7777.48");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1996.00,7797.70");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2335.09,7886.32");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3266.28,7974.74");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4917.32,7765.13");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=961.26,7729.73");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=1600.76,7525.07");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=3182.62,7583.25");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=3726.29,7742.54");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=4269.28,7727.52");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=5561.37,7724.06");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=2149.76,7386.94");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=2561.83,7468.63");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=4057.98,7337.51");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=4656.09,7488.06");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=5325.61,7413.02");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=5939.01,7486.25");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=577.64,7195.28");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=1218.48,7088.59");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=2907.03,7118.09");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=3345.38,7141.50");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=5132.34,7044.77");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=5642.51,7101.20");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=13.98,6908.00");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=825.30,6783.40");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=1505.40,6882.31");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=2106.61,6842.02");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=2670.99,6816.10");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=4594.22,6999.19");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=5959.22,6841.19");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=363.53,6535.13");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=2969.91,6558.49");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=3379.03,6703.68");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=3812.78,6581.19");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=5286.88,6550.42");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=710.87,6391.86");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=1162.33,6407.16");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=1786.05,6474.42");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=4311.10,6386.55");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=4660.82,6477.80");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=207.76,6149.52");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=2234.78,6177.60");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=3390.80,6231.54");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=5993.64,6208.87");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=984.70,5783.78");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=1423.03,5963.22");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=2279.67,5808.42");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=2824.96,5906.85");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=3197.17,5860.52");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=3599.80,5885.74");
    }

}

