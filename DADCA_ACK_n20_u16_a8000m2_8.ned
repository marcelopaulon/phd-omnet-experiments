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
        @display("i=misc/sensor;p=474.48,7856.42");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1614.05,7761.73");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2233.65,7970.25");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2855.25,7766.56");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3547.55,7861.89");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=3929.08,7868.07");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=4798.57,7808.74");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=5402.72,7944.23");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=189.25,7566.07");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=2006.26,7701.14");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=2491.46,7631.94");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=5131.65,7541.75");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=596.98,7478.59");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=1202.04,7498.40");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=2898.84,7389.86");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=3358.30,7418.13");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=4041.66,7477.35");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=4440.44,7479.60");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=1582.21,7067.65");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=2033.46,7248.33");
    }

}

