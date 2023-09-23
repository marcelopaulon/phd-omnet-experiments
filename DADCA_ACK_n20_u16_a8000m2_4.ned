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

network DADCA_ACK_n20_u16_a8000m2_4
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
        @display("i=misc/sensor;p=1780.02,7779.17");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=3632.24,7916.54");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=4081.02,7838.46");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=5670.68,7933.42");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=2451.33,7619.32");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=2900.82,7588.61");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=4508.59,7530.78");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=1398.92,7325.05");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=2247.33,7277.45");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=4011.60,7333.00");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=5273.03,7470.89");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=842.66,7092.09");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=1761.69,7070.15");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=3255.44,7129.47");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=4435.39,7132.81");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=4859.93,7094.59");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=5807.23,7165.88");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=270.28,6909.60");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=3779.56,6812.85");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=5421.79,6895.10");
    }

}

