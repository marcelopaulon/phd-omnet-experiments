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

network DADCA_ACK_n20_u16_a8000m2_1
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
        @display("i=misc/sensor;p=625.87,7890.49");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1461.65,7806.92");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2360.14,7833.37");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2780.96,7849.48");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3359.06,7956.28");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4092.20,7961.94");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=4562.79,7962.62");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=1167.58,7535.35");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=1922.44,7745.33");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=3786.74,7566.60");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=4211.10,7535.66");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=4621.65,7615.09");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=5104.29,7701.41");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=1781.05,7377.49");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=2187.22,7485.39");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=2739.51,7327.80");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=3148.90,7269.54");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=583.59,7227.31");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=929.44,7103.09");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=1493.15,7144.88");
    }

}

