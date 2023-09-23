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

network DADCA_ACK_n20_u4_a8000m2_7
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
        @display("i=misc/sensor;p=508.04,7821.43");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1536.65,7857.23");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2175.32,7961.66");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2766.09,7983.93");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4660.71,7918.50");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=2019.10,7567.61");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=2782.16,7536.02");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=3156.01,7672.93");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=3655.64,7670.06");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=961.91,7388.90");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=1702.09,7265.91");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=2394.80,7436.45");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=4820.21,7355.93");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=5344.60,7465.15");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=445.74,7155.64");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=2644.37,7044.41");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=642.07,6797.39");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=1215.57,6897.68");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=1912.77,6829.30");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=2299.02,6950.98");
    }

}

