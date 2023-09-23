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

network DADCA_ACK_n20_u16_a8000m2_8
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
        @display("i=misc/sensor;p=6.43,7917.85");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=660.04,7771.71");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1110.11,7840.13");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=1626.57,7848.02");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=2597.05,7988.06");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=3835.06,7790.74");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=4291.27,7820.64");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=5262.02,7852.77");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=2180.22,7596.11");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=2805.13,7610.63");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=3204.67,7548.47");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=4645.64,7743.11");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=5705.51,7712.78");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=1104.46,7399.08");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1541.09,7310.69");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=3691.88,7301.54");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=4165.92,7461.07");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=5208.08,7367.28");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=416.36,7234.45");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=789.51,7140.82");
    }

}

