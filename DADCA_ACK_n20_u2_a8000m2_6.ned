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

network DADCA_ACK_n20_u2_a8000m2_6
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
        @display("i=misc/sensor;p=457.20,7912.33");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1079.07,7999.75");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1686.23,7995.44");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2862.52,7930.06");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3809.21,7984.44");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4466.59,7820.10");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5754.97,7818.21");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=66.79,7632.12");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=1009.81,7587.32");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=1394.93,7703.64");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=1978.93,7639.60");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=2859.56,7550.89");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=3259.16,7624.79");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=4939.43,7654.37");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=5471.10,7577.21");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=599.22,7351.79");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=2410.89,7449.78");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=4229.63,7349.65");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=4643.44,7385.92");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=346.80,7074.73");
    }

}

