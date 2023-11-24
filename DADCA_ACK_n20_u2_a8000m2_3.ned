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

network DADCA_ACK_n20_u2_a8000m2_3
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
        @display("i=misc/sensor;p=461.35,7877.55");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1763.93,7922.08");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2328.19,7840.14");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3876.55,7883.39");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=5101.89,7925.13");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=5678.63,7807.12");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=1182.46,7704.72");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=3413.04,7592.19");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=4391.61,7733.76");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=90.56,7433.09");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=755.08,7266.39");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=2117.45,7466.15");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=2600.05,7479.67");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=3886.66,7443.09");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=4253.33,7343.60");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=4833.53,7347.90");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=5227.28,7382.68");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=370.96,7199.07");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=1431.32,7227.15");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=2435.13,7115.84");
    }

}

