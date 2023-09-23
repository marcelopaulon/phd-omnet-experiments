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

network DADCA_ACK_n20_u4_a8000m2_1
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
        @display("i=misc/sensor;p=864.25,7822.13");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1397.38,7928.33");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2449.12,7845.67");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2858.12,7849.26");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3971.34,7961.23");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4355.19,7929.04");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5435.49,7915.64");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=386.08,7737.93");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=1826.05,7580.68");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=2273.34,7516.48");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=3041.37,7529.50");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=3386.88,7745.61");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=4887.25,7656.63");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=503.49,7404.42");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1068.23,7489.16");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=2640.34,7328.63");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=3781.17,7398.47");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=4410.09,7456.90");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=4712.20,7276.55");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=5411.15,7295.82");
    }

}

