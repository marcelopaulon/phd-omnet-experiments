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

network DADCA_ACK_n50_u16_a8000m2_9
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
        @display("i=misc/sensor;p=1184.47,7767.44");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1570.44,7766.39");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2835.56,7929.02");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3657.45,7957.19");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4131.98,7772.34");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4894.31,7823.14");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=1885.77,7521.59");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=2434.78,7745.84");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=5202.17,7549.05");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=5777.18,7743.87");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=307.38,7316.67");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=734.01,7385.38");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=1476.86,7293.35");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=2224.96,7340.27");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=2919.31,7358.13");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=3268.38,7287.19");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=3704.97,7445.75");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=4059.42,7314.54");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=4594.22,7335.37");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=5976.14,7392.28");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=855.24,7047.81");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=2565.50,7154.48");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=3669.98,7021.42");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=5506.13,7100.19");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=1695.18,6889.53");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=2121.33,6832.48");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=2810.68,6870.60");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=3364.45,6840.75");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=4348.95,6778.27");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=4770.91,6923.32");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=5947.50,6864.63");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=199.24,6717.69");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=602.78,6674.99");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=3662.80,6548.96");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=5157.89,6559.59");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=5648.09,6643.84");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=201.06,6273.33");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=1147.59,6461.94");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=1891.67,6389.20");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=2434.83,6484.45");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=3088.16,6511.54");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=4091.76,6494.78");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=4633.12,6272.25");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=3337.23,6226.36");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=3683.89,6066.78");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=5294.82,6198.36");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=5829.62,6178.60");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=1449.08,5919.91");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=2253.57,5977.27");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=2861.41,5981.24");
    }

}

