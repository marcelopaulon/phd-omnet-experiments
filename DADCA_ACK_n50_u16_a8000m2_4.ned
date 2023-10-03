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

network DADCA_ACK_n50_u16_a8000m2_4
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
        @display("i=misc/sensor;p=1389.55,7792.81");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=2021.07,7933.56");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2700.01,7979.26");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3225.66,7995.49");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4129.69,7884.89");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4853.86,7942.57");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5241.37,7995.81");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=266.17,7635.31");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=691.94,7670.82");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=2977.18,7575.96");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=4503.46,7719.07");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=5406.21,7650.31");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=5881.22,7587.43");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=1351.56,7443.33");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1724.93,7425.68");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=2369.66,7364.32");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=3200.60,7269.28");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=3760.98,7468.57");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=4239.96,7263.85");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=5054.33,7469.29");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=357.59,7104.76");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=734.97,7119.04");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=1099.45,7177.08");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=2751.19,7188.58");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=4671.56,7054.11");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=5345.06,7203.95");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=1202.62,6804.72");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=1575.27,7003.70");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=2172.59,6995.69");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=2825.54,6789.36");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=3212.52,6816.25");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=3648.18,6950.85");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=5976.50,6904.70");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=29.20,6676.54");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=804.63,6581.30");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=1845.01,6528.01");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=4063.89,6632.50");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=5205.95,6607.43");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=5628.01,6631.56");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=1382.19,6388.45");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=2265.71,6503.58");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=2819.37,6339.03");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=4564.07,6496.55");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=541.64,6218.28");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=921.76,6028.89");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=2283.46,6122.91");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=2623.14,6033.53");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=3286.86,6173.60");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=3884.98,6042.67");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=4736.08,6034.85");
    }

}

