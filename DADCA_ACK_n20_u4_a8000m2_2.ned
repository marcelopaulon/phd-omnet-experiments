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

network DADCA_ACK_n20_u4_a8000m2_2
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
        @display("i=misc/sensor;p=532.82,7886.69");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1503.08,7900.78");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2101.48,7816.83");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2592.55,7909.12");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3252.76,7769.86");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=3678.48,7886.59");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=4393.46,7957.79");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=4806.35,7902.56");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=5224.36,7805.41");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=251.87,7598.01");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=922.81,7718.19");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=2827.71,7579.50");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=1412.63,7411.22");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=1953.88,7494.52");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=3364.93,7315.37");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=3919.81,7436.07");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=4510.38,7331.56");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=5064.65,7444.51");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=5546.55,7411.44");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=38.10,7119.45");
    }

}

